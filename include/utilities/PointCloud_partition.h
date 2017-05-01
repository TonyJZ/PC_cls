#ifndef _PointCloud_Partition_H_Create_by_Tony_2017_April_17_
#define _PointCloud_Partition_H_Create_by_Tony_2017_April_17_

#include <pcl/point_types.h>


template <typename PointT>
int pointcloud_partition_quadtree(pcl::PointCloud<PointT> &cloud, float blocksize, const char* pOutDir);




#include "utilities/impl/PointCloud_partition.hpp"
#endif
