// buildingDetection.cpp : Defines the entry point for the console application.
//
#define PCL_NO_PRECOMPILE

#include "stdafx.h"
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/octree/octree_pointcloud.h>

#include <boost/math/special_functions/round.hpp>

#include <pcl/ros/conversions.h>

#include <pcl_las/las_io.h>

#include "feature_extract/StatisticalOutlierRemoval.h"
#include "utilities/PointCloud_partition.h"

#include "segmentations/voxel_FC_graph.h"

#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>

struct EIGEN_ALIGN16 MyPoint2D              //定义点类型结构

{

	double x, y;                

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作

}EIGEN_ALIGN16;// 强制SSE对齐

//}

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint2D,// 注册点类型宏

	(double,x,x)

	(double,y,y)

	)

struct EIGEN_ALIGN16 PointXYZIL              //定义点类型结构

{
	PCL_ADD_POINT4D;
	float intensity;                
	uint32_t label;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作

}EIGEN_ALIGN16;// 强制SSE对齐

//}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIL,// 注册点类型宏

	(float,x,x)
	(float,y,y)
	(float,z,z)
	(float,intensity,intensity)
	(uint32_t,label,label)
	)

const unsigned char foreground_color[3] = {255, 0, 0};//{255, 255, 255};  red
const unsigned char background_color[3] = {255, 255, 255};//{255, 0, 0};  white

template <typename PointT> 
int Output_labled_pointcloud( pcl::PointCloud<PointT> &cloud, std::vector<int> &label_indices, const char *pFileName )
{
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	
	for(int i=0; i<cloud.size(); ++i)
	{
		pcl::PointXYZRGB ptc;
		ptc.x = *(cloud.points[i].data);
		ptc.y = *(cloud.points[i].data + 1);
		ptc.z = *(cloud.points[i].data + 2);
		ptc.r = background_color[0];
		ptc.g = background_color[1];
		ptc.b = background_color[2];
		out_cloud->points.push_back(ptc);
	}

	for(int i=0; i<label_indices.size(); ++i)
	{
		int id = label_indices[i];
		out_cloud->points[id].r = foreground_color[0];
		out_cloud->points[id].g = foreground_color[1];
		out_cloud->points[id].b = foreground_color[2];
	}

	writer.write<pcl::PointXYZRGB> (pFileName, *out_cloud, true);
	PCL_INFO("write: %s\n", pFileName);

	return 1;
}

int _tmain(int argc, _TCHAR* argv[])
{
	char *pFileName = argv[1];
	float vx, vy, vz;


	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 

	lasreader.read(pFileName,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double bbmin[3], bbmax[3];
	lasreader.getBoundingBox(bbmax, bbmin);

	double offset_ref_x = boost::math::iround( bbmin[0]);
	double offset_ref_y = boost::math::iround( bbmin[1]);
	double offset_ref_z = boost::math::iround( bbmin[2]);

	if(fabs(offset_ref_x) < 10000) offset_ref_x = 0;
	if(fabs(offset_ref_y) < 10000) offset_ref_y = 0;
	if(fabs(offset_ref_z) < 10000) offset_ref_z = 0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr org_pts(new pcl::PointCloud<pcl::PointXYZ>);
	org_pts->width = incloud->width;
	org_pts->height = incloud->height;

	for(int i=0; i<incloud->points.size(); i++)
	{
		pcl::PointXYZ pt;

		pt.x = incloud->points[i].x - offset_ref_x;
		pt.y = incloud->points[i].y - offset_ref_y;
		pt.z = incloud->points[i].z - offset_ref_z;

		org_pts->points.push_back(pt);
	}

	std::vector<int> filterd_indices;

	//1.预处理
//	pointcloud_partition_quadtree(*org_pts, 20, "E:/pointcloud/building_detection/");
//	return 0;


//	pcl_StatisticalOutlierRemoval (*org_pts, 50, 2.0, filterd_indices, true);

//	Output_labled_pointcloud(*org_pts, filterd_indices, "G:\\pointcloud\\building_detection\\outliers.pcd");
//	build_voxel_FC_graph(*org_pts, 2.0);


	//2.种子提取 is not necessary
//using namespace pcl;	

	/*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(org_pts);
	ne.setInputCloud(org_pts);
	ne.setSearchMethod(tree);
	ne.setKSearch(50);
	//ne->setRadiusSearch (0.03); 
	ne.compute(*pcNormal);	

	std::vector<int> indices;
	std::vector<float> distances;

	for (size_t idx = 0; idx < org_pts->size (); ++idx)
	{
		if (tree->nearestKSearch (org_pts->points[idx], 50, indices, distances) != 0)
		{
			Eigen::Vector4f xyz_centroid_;
			EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;
			computeMeanAndCovarianceMatrix (*org_pts, indices, covariance_matrix_, xyz_centroid_);

			Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix_);

			Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
			Eigen::Matrix3f V = es.pseudoEigenvectors();
		}

	}*/


	
	//3.graph cut
	pcl::UrbanRec::VoxelFCGraph<pcl::PointXYZ> vGraph;

	vGraph.setInputCloud(org_pts);

	//vGraph.setIndices();
	
	vx = vy = vz = 2.0;
	vGraph.setVoxelSize(vx, vy, vz);

	vGraph.compute_FC_Voxels();

	vGraph.voxel_features_extraction();

//	std::vector <pcl::PointIndices> clusters;
	vGraph.extract(/*clusters*/);

	vGraph.saveSegmentedFile("E:/pointcloud/building_detection/b2", "b2");
	return 0;
}

