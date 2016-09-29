#ifndef _3D_EDGE_DETECTION_Tony_Sep_15_2016_
#define _3D_EDGE_DETECTION_Tony_Sep_15_2016_

#include "segmentations/facade_detection.h"
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl_las/las_io.h>
#include <pcl/ros/conversions.h>

template <typename PointT>
int edge_detect_by_density(pcl::PointCloud<PointT> &cloud, float gridsize, float suppresionTh,
	pcl::PointCloud<PointT> &grid_cloud, std::vector<pcl::PointIndices> &label_indices)
{
	estimate_ProjectionDensity_horizontal_grid(cloud, gridsize, gridsize, grid_cloud);

	//suppress low density points
	for(int i=0; i<grid_cloud.points.size(); i++)
	{
		//会影响canny提取的结果
// 		if(grid_cloud.points[i].z < suppresionTh)
// 			grid_cloud.points[i].z = 0;
	}

	pcl::PointCloud<MyLasPoint>::Ptr mylas_cloud (new pcl::PointCloud<MyLasPoint>);
	mylas_cloud->width = grid_cloud.width;
	mylas_cloud->height = grid_cloud.height;
	mylas_cloud->points.resize(grid_cloud.points.size());

	for(int j=0; j<grid_cloud.points.size(); j++)
	{
		mylas_cloud->points[j].intensity = grid_cloud.points[j].z;
		mylas_cloud->points[j].x = grid_cloud.points[j].x;
		mylas_cloud->points[j].y = grid_cloud.points[j].y;
		mylas_cloud->points[j].z = grid_cloud.points[j].z;
		//unique_cloud->points[i].label = labels[i];
	}
	pcl::LASWriter las_writer;
	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
	pcl::toROSMsg(*mylas_cloud, *outCloud2);
	las_writer.write ("G:/temp/grid_points.las", *outCloud2);



	//edge detection
	pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setNormalSmoothingSize (10.0f);
	ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
	ne.setInputCloud (grid_cloud.makeShared());
	ne.compute (*normal);

	//OrganizedEdgeBase<PointXYZRGBA, Label> oed;
	//OrganizedEdgeFromRGB<PointXYZRGBA, Label> oed;
	//OrganizedEdgeFromNormals<PointXYZRGBA, Normal, Label> oed;
	pcl::OrganizedEdgeFromNormals<PointT, pcl::Normal, pcl::Label> oed;
	oed.setInputNormals (normal);
	oed.setInputCloud (grid_cloud.makeShared());
	oed.setDepthDisconThreshold (1.0);
	oed.setMaxSearchNeighbors (2.0);
	oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE /*| oed.EDGELABEL_RGB_CANNY*/);
	pcl::PointCloud<pcl::Label> labels;
//	std::vector<PointIndices> label_indices;
	oed.compute (labels, label_indices);

	return 0;
}





#endif