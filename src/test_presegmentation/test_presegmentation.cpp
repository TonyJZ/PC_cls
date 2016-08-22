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

#include <pcl/ros/conversions.h>

#include <pcl_las/las_io.h>
//#include "feature_extract/features_point.h"
#include "segmentations/duplicate_removing.h"
#include "segmentations/clustering_density_peaks.h"
#include "segmentations/height_slicing.h"


//#define PCL_

//namespace tpcs{   //tony point cloud segmentation


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


int pointFilter_ProgressiveMorphological ();
int pointFilter_ApproximateProgressiveMorphological ();

double euclideanDistance2(MyPoint2D &p1, MyPoint2D &p2)
{
	return (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y);
}

double euclideanDistance(MyPoint2D &p1, MyPoint2D &p2)
{
	return sqrt(euclideanDistance2(p1, p2));
}

typedef struct
{
	int count;
	float    density;
} PT_Local_Density ;

/*
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>
int ProjectionDensityEstimation_XYPlane(pcl::PointCloud<pcl::PointXY>::Ptr cloud, std::vector<PT_Local_Density> &lsdensity, float radius)
{
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud (cloud);

	pcl::PointXY searchPoint;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	//	float radius = 0.5;

	pcl::PointCloud<pcl::PointXY>::iterator iter;

	iter = cloud->points.begin();

	for( ; iter != cloud->points.end(); iter++)
	{
		PT_Local_Density  dens;
		dens.density = 0.0;
		dens.count = 0;
		//float dens = 0.0;

		searchPoint = *iter;
		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1 )
		{
			float sumdis=0;
			for (size_t i = 1; i < pointIdxRadiusSearch.size (); ++i)
			{
				if(pointRadiusSquaredDistance[i] == 0)
					continue;

				sumdis += 1.0 / sqrt(pointRadiusSquaredDistance[i]); //euclideanDistance(cloud->points[ pointIdxRadiusSearch[i] ], searchPoint);
			}

			if(_finite(sumdis)==false)
			{
				std::cout << "error"<< std::endl;
			}
			//dens = (pointIdxRadiusSearch.size ()-1) / sumdis;
			dens.density = sumdis;
		}
		else
		{
			dens.density = 0;
		}

		dens.count = pointIdxRadiusSearch.size ()-1;
		lsdensity.push_back(dens);
	}

	return 1;
}

void test_ProjectionDensityEstimation_XYPlane()
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read("G:/pointcloud/Dundas_University/CSF_fitered/off-ground points.las",
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);


	pcl::PointCloud<pcl::PointXY>::Ptr incloud_xy (new pcl::PointCloud<pcl::PointXY>);
	std::vector<PT_Local_Density> densities;

	incloud_xy->points.resize(incloud->points.size());

	double xoffset, yoffset;
	xoffset = (uint32_t)(incloud->points[0].x);
	yoffset = (uint32_t)(incloud->points[0].y);
	for (size_t i = 0; i < incloud->points.size (); ++i)
	{
		incloud_xy->points[i].x = incloud->points[i].x - xoffset;
		incloud_xy->points[i].y = incloud->points[i].y - yoffset;
	}

	//计算投影密度
	float searchR = 0.5;
	ProjectionDensityEstimation_XYPlane(incloud_xy, densities, searchR);

	//统计密度直方图
	float max_den, min_den;
	max_den = 0; 
	min_den = std::numeric_limits<double>::max();

	//	size_t i;
	int max_count = 0;
	int min_count = 1000000;

	for(size_t i=0; i<densities.size(); i++)
	{
		if(densities[i].count == 0)
			continue;

		if(max_den < densities[i].density) max_den = densities[i].density;
		if(min_den > densities[i].density) min_den = densities[i].density;

		if(max_count < densities[i].count)	max_count = densities[i].count;
		if(min_count > densities[i].count)	min_count = densities[i].count;
	}

	int interval = 100;
	std::vector<int> den_histgram;
	den_histgram.resize(interval+1);
	den_histgram.assign(interval+1, 0);

	float span = (max_den + 1e-3 - min_den)/interval;
	for(size_t i=0; i<densities.size(); i++)
	{
		if(densities[i].count == 0)
		{
			den_histgram[0]++;

			incloud->points[i].intensity = 0;

			continue;
		}

		int j;
		j = (int)((densities[i].density - min_den)/span)+1;

		if(j<0 || j>=interval+1)
		{
			// 			float err = densities[i];
			// 			err = (densities[i]-min_den)/span;
			assert(false);
		}

		den_histgram[j]++;

		incloud->points[i].intensity = densities[i].density; 
			(densities[i].density-min_den)/span;
		densities[i].count;
	}

	FILE *fp=NULL;
	fp = fopen("G:\\pointcloud\\Dundas_University\\prjDen_histogram", "wt");

	fprintf(fp, "%f, %d\n", 1.0/searchR, den_histgram[0]);
	for(size_t i=1; i<den_histgram.size(); i++)
	{

		fprintf(fp, "%f, %d\n", min_den+i*span, den_histgram[i]);
	}


	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
	pcl::toROSMsg(*incloud, *outCloud2);	
	pcl::LASWriter writer;

	writer.write ("G:/pointcloud/Dundas_University/CSF_fitered/off-ground points_prjDen_linearEnhance.las", *outCloud2);
}
*/

template <typename PointT>
int slice_segmentation(const pcl::PointCloud<PointT> &cloud, double h_interval, double neighborRadius, double alpha, double beta,
	std::vector<uint32_t> &labels)
{

	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
// 	Eigen::Vector4f origin;
// 	Eigen::Quaternionf rot;
// 	int fv;
// 	pcl::PointCloud<PointXYZIL>::Ptr incloud (new pcl::PointCloud<PointXYZIL>); 

	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read("G:\\pointcloud\\Dundas_University\\filtered_by_Chen\\ngp.las",
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);
	
	double offset_x = boost::math::iround(incloud->points[0].x);
	double offset_y = boost::math::iround( incloud->points[0].y);
	double offset_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_x) < 10000) offset_x = 0;
	if(fabs(offset_y) < 10000) offset_y = 0;
	if(fabs(offset_z) < 10000) offset_z = 0;

// 	pcl::io::loadPCDFile("G:\\pointcloud\\Dundas_University\\filtered_by_Chen\\ngp.pcd", 
// 	 		*incloud);

	pcl::PointCloud<MyLasPoint>::Ptr unique_cloud (new pcl::PointCloud<MyLasPoint>); 
	remove_duplicate_points(*incloud, *unique_cloud);


	std::vector<uint32_t> global_labels;     //labels for the whole scene
	std::vector<PointIndices> slice_indices;
	height_slicing(*unique_cloud, 3.0, slice_indices);

	pcl::PointCloud<MyLasPoint>::Ptr slice_points (new pcl::PointCloud<MyLasPoint>);
	pcl::ExtractIndices<MyLasPoint> extract;
	

	int islice = 0;
	for(std::vector<PointIndices>::iterator itIndices=slice_indices.begin(); itIndices!=slice_indices.end(); ++itIndices, ++islice)
	{
		pcl::PointIndicesPtr sIdx (new pcl::PointIndices);
		sIdx->indices = *itIndices;
		extract.setInputCloud (unique_cloud);
		extract.setIndices (sIdx);
		extract.filter (*slice_points);


		std::vector<int> cluster_centers;
		std::vector<uint32_t> slabels;	//labels for current slice points 
		std::vector<double> rhos;

		//extract features 
		pcl::PointCloud<pcl::PointXYZ>::Ptr features(new pcl::PointCloud<pcl::PointXYZ>);
		features->width = slice_points->width;
		features->height = slice_points->height;

		for(int k=0; k < slice_points->points.size(); k++)
		{
			pcl::PointXYZ pt;

			pt.x = slice_points->points[k].x - offset_x;
			pt.y = slice_points->points[k].y - offset_y;
			pt.z = slice_points->points[k].z - offset_z;

			features->points.push_back(pt);
		}

		//clustering in feature space
		clustering_by_density_peaks(*features, 10.0, 3.0, 0.05, cluster_centers, slabels, rhos);

		//cluster center
		pcl::PointIndicesPtr center_indices (new pcl::PointIndices);
		center_indices->indices = cluster_centers;

		pcl::PointCloud<MyLasPoint>::Ptr cluster_center (new pcl::PointCloud<MyLasPoint>);
		//pcl::ExtractIndices<PointXYZIL> extract;
		extract.setInputCloud (slice_points);
		extract.setIndices (center_indices);
		extract.filter (*cluster_center);

		//output 
		pcl::LASWriter las_writer;
		pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
		pcl::PointCloud<MyLasPoint>::Ptr out_cloud(new pcl::PointCloud<MyLasPoint>);

		char sCenterName[256], sSegmentsName[256];

		sprintf(sCenterName, "G:\\pointcloud\\Dundas_University\\filtered_by_Chen\\sCenters%d.las", islice);
		sprintf(sSegmentsName, "G:\\pointcloud\\Dundas_University\\filtered_by_Chen\\segment_rhos%d.las", islice);

		out_cloud->height = cluster_center->height;
		out_cloud->width = cluster_center->width;
		out_cloud->points.resize(cluster_center->points.size());

		for(int i=0; i<cluster_center->points.size(); i++)
		{
			out_cloud->points[i].x = cluster_center->points[i].x;
			out_cloud->points[i].y = cluster_center->points[i].y;
			out_cloud->points[i].z = cluster_center->points[i].z;
			//		out_cloud->points[i].classification = labels[i];
			//		out_cloud->points[i].intensity = rhos[i];
			//		unique_cloud->points[i].label = labels[i];
		}

		pcl::toROSMsg(*out_cloud, *outCloud2);	

		las_writer.write (sCenterName, *outCloud2);


		out_cloud->height = slice_points->height;
		out_cloud->width = slice_points->width;
		out_cloud->points.resize(slice_points->points.size());

		for(int i=0; i<slice_points->points.size(); i++)
		{
			out_cloud->points[i].x = slice_points->points[i].x;
			out_cloud->points[i].y = slice_points->points[i].y;
			out_cloud->points[i].z = slice_points->points[i].z;
			out_cloud->points[i].classification = slabels[i];
			out_cloud->points[i].intensity = rhos[i];
			slice_points->points[i].label = slabels[i];
		}


		pcl::toROSMsg(*out_cloud, *outCloud2);	

		las_writer.write (sSegmentsName, *outCloud2);
	}

//	writer.write<pcl::PointXYZL> ("G:\\pointcloud\\Dundas_University\\filtered_by_Chen\\segment_cdp.pcd", *unique_cloud, false);
	return 0;
}

//note: this method is time-consuming, suggest to use ApproximateProgressiveMorphological
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

