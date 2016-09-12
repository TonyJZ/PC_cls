#ifndef _FACADE_DETECTION_H_Tony_Sep_08_2016_
#define _FACADE_DETECTION_H_Tony_Sep_08_2016_


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>

template <typename PointT>
int estimate_ProjectionDensity_XYPlane(pcl::PointCloud<PointT> &cloud, float radius, std::vector<float> &densities)
{
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (cloud.makeShared());

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	//	float radius = 0.5;

	pcl::PointCloud<PointT>::iterator iter;

	iter = cloud.points.begin();

	for( ; iter != cloud.points.end(); iter++)
	{
		int nn = kdtree.radiusSearch (*iter, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if ( nn > 1 )
		{
			densities.push_back(nn-1);
		}
		else
		{
			densities.push_back(0);
		}

	}

	return 1;
}



#endif