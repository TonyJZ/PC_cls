#ifndef _Point_Cloud_Feature_Extract_H_ZJ_2016_08_05_
#define _Point_Cloud_Feature_Extract_H_ZJ_2016_08_05_


#include <pcl/pcl_exports.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


//note: PointT only use the X-Y 2D coordinates
	template <typename PointT> int
	ProjectionDensityEstimation_XYPlane(pcl::PointCloud<PointT>::Ptr cloud, std::vector<float> &lsdensity, float radius)
{
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (cloud);

	PointT searchPoint;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	//	float radius = 0.5;

	pcl::PointCloud<PointT>::iterator iter;

	iter = cloud->points.begin();

	for( ; iter != cloud->points.end(); iter++)
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

//#include <feature_extract/impl/features_point.hpp>
#endif