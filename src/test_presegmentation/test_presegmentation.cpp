// test_presegmentation.cpp : Defines the entry point for the console application.
//
#define PCL_NO_PRECOMPILE

#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

#include <pcl/io/las_io.h>

#include <pcl/ros/conversions.h>


//#define PCL_

//namespace tpcs{   //tony point cloud segmentation
struct EIGEN_ALIGN16 MyLasPoint              //定义点类型结构

{

	double x, y, z;                

	float intensity;

	uint32_t classification;

	uint32_t label;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作

}EIGEN_ALIGN16;// 强制SSE对齐

//}

POINT_CLOUD_REGISTER_POINT_STRUCT(MyLasPoint,// 注册点类型宏

	(double,x,x)

	(double,y,y)

	(double,z,z)

	(float,intensity,intensity)

	(uint32_t, classification, classification)

	(uint32_t, label, label)

)



int pointFilter_ProgressiveMorphological ();
int pointFilter_ApproximateProgressiveMorphological ();


int _tmain(int argc, _TCHAR* argv[])
{

//	pointFilter_ProgressiveMorphological();

//	pointFilter_ApproximateProgressiveMorphological();

	/* test pcd IO  */
// 	pcl::PointCloud<MyPointXYZICL>::Ptr cloud (new pcl::PointCloud<MyPointXYZICL>);
// 	pcl::PCDReader reader;
// 	reader.read<MyPointXYZICL> ("ALTM_Strip_Dundas.pcd", *cloud);

	
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 
	pcl::PointCloud<MyLasPoint>::Ptr incloud1 (new pcl::PointCloud<MyLasPoint>); 

	lasreader.read("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas.las",
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

// 	lasreader.read("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas_write.las",
// 		*inCloud2, incloud1->sensor_origin_, incloud1->sensor_orientation_, fv);
// 	pcl::fromPCLPointCloud2 (*inCloud2, *incloud1);

	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
	pcl::toROSMsg(*incloud, *outCloud2);	
	pcl::LASWriter writer;
	
	writer.write ("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas_write.las", *outCloud2);


	return 0;
}


int pointFilter_ProgressiveMorphological ()
{
//  	pcl::PointCloud<MyLasPoint>::Ptr cloud (new pcl::PointCloud<MyLasPoint>);
//  	pcl::PointCloud<MyLasPoint>::Ptr cloud_filtered (new pcl::PointCloud<MyLasPoint>);
	pcl::PointIndicesPtr ground (new pcl::PointIndices);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>); 
	//reader.read<pcl::PointXYZI> ("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas.pcd", *icloud);

	pcl::PCLPointCloud2::Ptr myCloud(new pcl::PCLPointCloud2);
// 	pcl::io::loadPCDFile("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas.pcd", 
// 		*myCloud, icloud->sensor_origin_, icloud->sensor_orientation_);
// 
// 	pcl::fromPCLPointCloud2 (*myCloud, *cloud);


	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
	pmf.setInputCloud (cloud);
	pmf.setMaxWindowSize (20);
	pmf.setSlope (1.0f);
	pmf.setInitialDistance (0.5f);
	pmf.setMaxDistance (3.0f);
	pmf.extract (ground->indices);

	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (ground);
//	extract.filter (*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
//	std::cerr << *cloud_filtered << std::endl;

// 	pcl::PCDWriter writer;
// 	writer.write<MyLasPoint> ("G:\\pointcloud\\Dundas_University\\ALTM_Strip_Dundas_filtered.pcd", *cloud_filtered, false);

	// Extract non-ground returns
// 	extract.setNegative (true);
// 	extract.filter (*cloud_filtered);
// 
// 	std::cerr << "Object cloud after filtering: " << std::endl;
// 	std::cerr << *cloud_filtered << std::endl;
// 
// 	writer.write<pcl::PointXYZ> ("G:\\pointcloud\\Dundas_University\\samp11-utm_object.pcd", *cloud_filtered, false);

	return (0);
}

int pointFilter_ApproximateProgressiveMorphological ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground (new pcl::PointIndices);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ> ("ALTM_Strip_Dundas.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud (cloud);
	pmf.setMaxWindowSize (35);
	pmf.setSlope (1.0f);
	pmf.setInitialDistance (0.5f);
	pmf.setMaxDistance (3.0f);
	pmf.extract (ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (ground);
	extract.filter (*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("ALTM_Strip_Dundas_ground.pcd", *cloud_filtered, false);

	// Extract non-ground returns
	extract.setNegative (true);
	extract.filter (*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	writer.write<pcl::PointXYZ> ("ALTM_Strip_Dundas_object.pcd", *cloud_filtered, false);

	return 0;
}