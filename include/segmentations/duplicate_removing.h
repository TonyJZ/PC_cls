#ifndef _Duplicate_Points_Removing_H_Tony_2016_Aug_10_
#define _Duplicate_Points_Removing_H_Tony_2016_Aug_10_

#include <algorithm>
#include <pcl/point_types.h>

template <typename PointT>
bool comparePoint(PointT p1, PointT p2)
{
	if (p1.x != p2.x)
		return p1.x > p2.x;
	else if (p1.y != p2.y)
		return  p1.y > p2.y;
	else
		return p1.z > p2.z;
}

template <typename PointT>
bool equalPoint(PointT p1, PointT p2)
{
	// floating point comparisons
	float eps = 1e-6;

	if (fabs(p1.x -p2.x) < eps && fabs(p1.y - p2.y) < eps && fabs(p1.z - p2.z) < eps)
		return true;

	return false;
}

template <typename PointT>
int remove_duplicate_points(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &unique_cloud)
{
	std::sort(cloud.points.begin(), cloud.points.end(), comparePoint<PointT>);
	auto unique_end = std::unique(cloud.points.begin(), cloud.points.end(), equalPoint<PointT>);

	unique_cloud.points.assign(cloud.points.begin(), unique_end);

	return 0;
}



#endif