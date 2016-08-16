// feature_extract.cpp : Defines the exported functions for the DLL application.
//

//#include "stdafx.h"
//#include "feature_extract/features_point.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//
template <typename PointT> 
int ProjectionDensityEstimation_XYPlane(const pcl::PointCloud<PointT> &cloud, 
	std::vector<float> &lsdensity, float radius)
{
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (cloud);

	PointT searchPoint;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

//	float radius = 0.5;

	pcl::PointCloud<PointT>::iterator iter;

	iter = cloud.begin();

	for( ; iter != cloud.end(); iter++;)
	{
		float dens = 0.0;

		searchPoint = *iter;
		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			float sumdis=0;
			for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			{
				sumdis += euclideanDistance(cloud->points[ pointIdxRadiusSearch[i] ], searchPoint);
			}

			dens = pointIdxRadiusSearch.size () / sumdis;
		}
		
		lsdensity.push_back(dens);
	}

	return 1;
}
