#ifndef _Point_Cloud_Register_H_Tony_Sep_15_2016_
#define _Point_Cloud_Register_H_Tony_Sep_15_2016_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace P_CLS 
{
	enum Trans_Type
	{
		Non_Trans = 0,
		Rigid2D = 1,
		Rigid3D = 2
	};


	typedef struct  
	{
		Trans_Type type;
		double coeffs[32];

	}Transform_params;

}



template <typename PointT>
int simple_ICP (pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &tar_cloud, pcl::PointCloud<PointT> &final_cloud,
	Eigen::Matrix4f &trans_param, double &fitScore)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputCloud(org_cloud.makeShared());
	icp.setInputTarget(tar_cloud.makeShared());
	icp.align(final_cloud);
	
	fitScore = icp.getFitnessScore();
	
	trans_param = icp.getFinalTransformation();

	return (0);
}



#endif