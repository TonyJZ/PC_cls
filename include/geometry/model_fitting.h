#ifndef _GEOMETRY_MODEL_FITTING_METHODS_H_ZJ_2016_AUG_8_
#define _GEOMETRY_MODEL_FITTING_METHODS_H_ZJ_2016_AUG_8_

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

template <typename PointT>
int plane_fitting_SAC (pcl::PointCloud<PointT> &cloud,std::vector<int> *indices, double dTh,
	pcl::ModelCoefficients &coefficients, std::vector<int> &inliers)
{
	
	inliers.clear();

// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
 	pcl::PointIndices::Ptr model_indices (new pcl::PointIndices);

	pcl::PointCloud<PointT> sel_pts;
	if(indices)
	{
		pcl::copyPointCloud (cloud, *indices, sel_pts);
	}
	else
		pcl::copyPointCloud (cloud, sel_pts);
	
	

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (dTh);

	seg.setInputCloud (sel_pts.makeShared());
	seg.segment (*model_indices, coefficients);

	if (inliers.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	else
	{
		for(int i=0; i<model_indices->indices.size(); ++i)
		{
			if(indices)
				inliers.push_back(indices->at(model_indices->indices[i]));
			else
				inliers.push_back(model_indices->indices[i]);
		}

	}

	return (0);
}

template <typename PointT>
int line_fitting_SAC (pcl::PointCloud<PointT> &cloud,std::vector<int> *indices, double dTh,
	pcl::ModelCoefficients &coefficients, std::vector<int> &inliers)
{
	inliers.clear();

	// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr model_indices (new pcl::PointIndices);

	pcl::PointCloud<PointT> sel_pts;
	if(indices)
	{
		pcl::copyPointCloud (cloud, *indices, sel_pts);
	}
	else
		pcl::copyPointCloud (cloud, sel_pts);

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (dTh);

	seg.setInputCloud (sel_pts.makeShared());
	seg.segment (*model_indices, coefficients);

	if (inliers.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	else
	{
		for(int i=0; i<model_indices->indices.size(); ++i)
		{
			if(indices)
				inliers.push_back(indices->at(model_indices->indices[i]));
			else
				inliers.push_back(model_indices->indices[i]);
		}

	}

	return (0);
}

#endif