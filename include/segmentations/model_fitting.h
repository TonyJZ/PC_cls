#ifndef _GEOMETRY_MODEL_FITTING_METHODS_H_ZJ_2016_AUG_8_
#define _GEOMETRY_MODEL_FITTING_METHODS_H_ZJ_2016_AUG_8_

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

template <typename PointT> int 
	plane_fitting_SAC (pcl::PointCloud<PointT> &cloud, double dTh,
	pcl::ModelCoefficients &coefficients, pcl::PointIndices::Ptr inliers)
{
	

// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
// 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (dTh);

	seg.setInputCloud (&cloud);
	seg.segment (*inliers, coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	for (size_t i = 0; i < inliers->indices.size (); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;

	return (0);
}



#endif