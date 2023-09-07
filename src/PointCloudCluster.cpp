#include <chrono>
#include <omp.h>
#include <string.h>
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <perception_msgs/msg/point_cluster.hpp>
#include <perception_msgs/msg/point_cluster_vec.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include "PointCloudTool.hpp"
#include "tool.hpp"
#include "rclcpp/rclcpp.hpp"

class PointCloudCluster: public rclcpp::Node {
public:
    PointCloudCluster(std::string name):Node(name) {
        mcloud = new pointCloud;
        using std::placeholders::_1;
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points_80", 1, std::bind(&PointCloudCluster::execute_cluster, this, std::placeholders::_1));
        pub = this->create_publisher<perception_msgs::msg::PointClusterVec>("sam_point_cluster", 1);
        std::cout<<"init PointCloudCluster"<<std::endl;
    }
    void execute_cluster(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::cout<<"load_msg:"<<msg->header.frame_id<<" ts:"<<msg->header.stamp.sec<<std::endl;
        auto start = std::chrono::steady_clock::now();
        perception_msgs::msg::PointClusterVec point_cluster_vec;

	    point_cluster_vec.header.set__stamp(msg->header.stamp);
        point_cluster_vec.header.set__frame_id("sam_point_cluster");
        init();
        load_input(msg);
        auto input_t = std::chrono::steady_clock::now();
        newtool.NewGroundSeg(mcloud);
        auto groundseg_t = std::chrono::steady_clock::now();
        std::vector< std::vector< std::vector<pointX> > > AllClusters = newtool.PointCloudCluster(mcloud);
        vector<OneCluster> FinalCLuster= newtool.CombineClusterResult(&AllClusters);
        auto cluster_t = std::chrono::steady_clock::now();
        assign_label(AllClusters, FinalCLuster, point_cluster_vec);

        std::cout<<"out_msg, ts:"<<point_cluster_vec.header.stamp.sec<<", frame_id: "<<point_cluster_vec.header.frame_id<<std::endl;
        auto result_t = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::milliseconds>(result_t - start).count();
        auto i_latency = std::chrono::duration_cast<std::chrono::milliseconds>(input_t - start).count();
        auto g_latency = std::chrono::duration_cast<std::chrono::milliseconds>(groundseg_t - input_t).count();
        auto c_latency = std::chrono::duration_cast<std::chrono::milliseconds>(cluster_t - groundseg_t).count();
        auto r_latency = std::chrono::duration_cast<std::chrono::milliseconds>(result_t - groundseg_t).count();
        RCLCPP_INFO(this->get_logger(), "All Cost Time %d ms, step1: %d ms, step2: %d ms, step3: %d ms, step4: %d ms", latency, i_latency, g_latency, c_latency, r_latency);
        pub->publish(point_cluster_vec);

    }

private:
    pointCloud *mcloud;
    PointcloudTool newtool;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Publisher<perception_msgs::msg::PointClusterVec>::SharedPtr pub;
    int point_size;
    

    void init() {
	point_size = 0;
        for (int l = 0; l < LINE; l++) {
            for (int c = 0; c < CIRCLEMAXLEN; c++) {
                mcloud->mptclout[l][c].x = 0;
                mcloud->mptclout[l][c].y = 0;
                mcloud->mptclout[l][c].z = 0;
                mcloud->mptclout[l][c].d = 0;
                mcloud->mptclout[l][c].isused = 0;  
                mcloud->mptclout[l][c].type = 0;
            }
        }
        mcloud->circlelen = 1800;
    }

    void load_input(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud* scan = new sensor_msgs::msg::PointCloud();
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, *scan);
        size_t N = scan->points.size();
        int line, circlept;
        float x, y, z;
        for (auto i = 0; i < N; i++) {
            x = scan->points[i].x;
            if (x<=0) continue;
            y = scan->points[i].y;
            z = scan->points[i].z;
	        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            // circlept=int(atan2(y, x)*900/M_PI+900);
            circlept=int(faster_atan2(y, x)*900/M_PI+900);
            if (circlept >= 1100 || circlept <= 700) continue;
            // circlept=int(fastAtan2(y, x)*5);
            line=scan->channels[1].values[i];
            if (line>=LINE) continue;
            mcloud->mptclout[line][circlept].x = x;
            mcloud->mptclout[line][circlept].y = y;
            mcloud->mptclout[line][circlept].z = z;
            mcloud->mptclout[line][circlept].isused=1;
            mcloud->mptclout[line][circlept].type=20;
	        point_size += 1;
        }
	    delete scan;
    }


    void assign_label(std::vector< std::vector< std::vector<pointX>>>& AllClusters, vector<OneCluster>& FinalCLuster, perception_msgs::msg::PointClusterVec& pc_vec) {
        int vec_size = 0;
        // std::vector<perception_msgs::msg::PointCluster> vec;
        auto& vec = pc_vec.vec;
        // vec.resize(point_size);
        vec.reserve(point_size);
        for (int i = 0; i < FinalCLuster.size(); i++) {
            auto& one_cluster = FinalCLuster[i];
            auto& point_indexes = one_cluster.PointIndex;
            for (auto& point_index: point_indexes) {
                int r_index = point_index.x;
                int c_index = point_index.y;
                auto& sub_cluster = AllClusters[r_index][c_index];
                for (auto& point: sub_cluster) {
                    perception_msgs::msg::PointCluster point_cluster;
                    point_cluster.set__x(point.x);
                    point_cluster.set__y(point.y);
                    point_cluster.set__z(point.z);
                    point_cluster.set__label(i);
                    vec.push_back(point_cluster);
                    vec_size += 1;
                }
            }
        }
        // vec.resize(vec_size);
        // pc_vec.set__vec(vec);
        pc_vec.set__size(vec_size);
    }
};

int main(int argc, char **argv)
{
 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCluster>("PointCloudCluster");
    rclcpp::spin(node);
   
    rclcpp::shutdown();
 
    return 0;
}

