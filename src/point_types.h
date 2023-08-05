/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */


#include <pcl/point_types.h>

namespace Obstacle2DNameSpace
{
class pointcloud
{
public:
	float x;
	float y;
	float z;
	float d;
	bool isaddground;//后面填补地面点标记
	bool ignore;//地面点标记
	bool isground;//地面点标记
	bool isroad;//法向量选点标记
	bool isreachable;//法向量选点标记
	int gridx;//在占据栅格图的位置
	int gridy;//在占据栅格图的位置
  int obj_label;
};

  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct pointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 //< laser intensity reading
    uint16_t ring;                      //< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(Obstacle2DNameSpace::pointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

