// test_presegmentation.cpp : Defines the entry point for the console application.
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


#include <pcl/ros/conversions.h>

#include <pcl_las/las_io.h>
//#include "feature_extract/features_point.h"
#include "segmentations/duplicate_removing.h"
#include "segmentations/clustering_density_peaks.h"
#include "segmentations/height_slicing.h"
//#include "segmentations/PCL_Segmentations.h"

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

#include "segmentations/DBSCAN.h"

template <typename PointT>
int clustering_by_DBSCAN(const pcl::PointCloud<PointT> &cloud, double Eps, int MinPts,
	std::vector<int> &cluster_center_idx, std::vector<uint32_t> &labels, std::vector<double> &rhosout)
{
	int nSamples = cloud.points.size();

	point_t *points = new point_t[nSamples];

	for(int i=0; i<nSamples; i++)
	{
		points[i].x = cloud.points[i].x;
		points[i].y = cloud.points[i].y;
		points[i].z = cloud.points[i].z;
		points[i].cluster_id = UNCLASSIFIED;
	}


	dbscan( points, nSamples, Eps, MinPts,  euclidean_dist);


	labels.resize(nSamples);
	for(int i=0; i<nSamples; i++)
	{
		labels[i] = points[i].cluster_id;
	}

	rhosout.assign(nSamples, 0);

	if(points)	delete[] points;   points = NULL;

	return 0;
}

#include "segmentations/OPTICS.h"
template <typename PointT>
int clustering_by_optics(const pcl::PointCloud<PointT> &cloud, double Eps, int MinPts,
	std::vector<int> &cluster_center_idx, std::vector<uint32_t> &labels, std::vector<double> &rhosout)
{
	int nSamples = cloud.points.size();

	opt_point_t *points = new opt_point_t[nSamples];

	for(int i=0; i<nSamples; i++)
	{
		points[i].x = cloud.points[i].x;
		points[i].y = cloud.points[i].y;
		points[i].z = cloud.points[i].z;
		points[i].cluster_id = UNCLASSIFIED;
	}


	optics( points, nSamples, Eps, MinPts,  euclidean_dist);


	labels.resize(nSamples);
	for(int i=0; i<nSamples; i++)
	{
		labels[i] = points[i].cluster_id;
	}

	rhosout.assign(nSamples, 0);

	if(points)	delete[] points;   points = NULL;

	return 0;
}

#include "segmentations/facade_detection.h"
void remove_facadePoints(char *pFileName, float radius, float denTh, char *pOutName)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr cloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read(pFileName, *inCloud2, cloud->sensor_origin_, cloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *cloud);

	pcl::PointCloud<pcl::PointXY>::Ptr features(new pcl::PointCloud<pcl::PointXY>);
	features->width = cloud->width;
	features->height = cloud->height;

	double offset_x = boost::math::iround(cloud->points[0].x);
	double offset_y = boost::math::iround( cloud->points[0].y);
	double offset_z = boost::math::iround( cloud->points[0].z);

	if(fabs(offset_x) < 10000) offset_x = 0;
	if(fabs(offset_y) < 10000) offset_y = 0;
	if(fabs(offset_z) < 10000) offset_z = 0;

	for(int k=0; k < cloud->points.size(); k++)
	{
		pcl::PointXY pt;

		pt.x = cloud->points[k].x - offset_x;
		pt.y = cloud->points[k].y - offset_y;
//		pt.z = cloud->points[k].z - offset_z;

		features->points.push_back(pt);
	}

	std::vector<float> densities;
	estimate_ProjectionDensity_XYPlane(*features, radius, densities);

	pcl::PointIndicesPtr non_facades (new pcl::PointIndices);
	for(size_t i=0; i<densities.size(); i++)
	{
		if(densities[i] < denTh)
			non_facades->indices.push_back(i);
	}

	pcl::ExtractIndices<MyLasPoint> extract;
	pcl::PointCloud<MyLasPoint>::Ptr slice_points (new pcl::PointCloud<MyLasPoint>);

	extract.setInputCloud (cloud);
	extract.setIndices (non_facades);
	extract.filter (*slice_points);

	char output_name[256];

	sprintf(output_name, "%s", pOutName);

	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
	if(slice_points->points.size()>0)
		pcl::toROSMsg(*slice_points, *outCloud2);	

	pcl::LASWriter las_writer;
	las_writer.write (output_name, *outCloud2);
}

int segment_whole_pointcloud(char *pInputName, double neighborRadius, double dc, double k_percent, int min_reachablepts,
	char *pOutputName)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read(pInputName,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_x = boost::math::iround(incloud->points[0].x);
	double offset_y = boost::math::iround( incloud->points[0].y);
	double offset_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_x) < 10000) offset_x = 0;
	if(fabs(offset_y) < 10000) offset_y = 0;
	if(fabs(offset_z) < 10000) offset_z = 0;


	pcl::PointCloud<MyLasPoint>::Ptr unique_cloud (new pcl::PointCloud<MyLasPoint>); 
	remove_duplicate_points(*incloud, *unique_cloud);


	std::vector<int> peaks_indices;
	std::vector<uint32_t> slabels;	//labels for current slice points 
	std::vector<double> rhos;

	//extract features 
	pcl::PointCloud<pcl::PointXYZ>::Ptr features(new pcl::PointCloud<pcl::PointXYZ>);
	features->width = unique_cloud->width;
	features->height = unique_cloud->height;

	for(int k=0; k < unique_cloud->points.size(); k++)
	{
		pcl::PointXYZ pt;

		pt.x = unique_cloud->points[k].x - offset_x;
		pt.y = unique_cloud->points[k].y - offset_y;
		pt.z = unique_cloud->points[k].z - offset_z;

		features->points.push_back(pt);
	}

	//clustering in feature space
	clustering_by_density_peaks_modified(*features, neighborRadius, dc, k_percent, min_reachablepts, 
		peaks_indices, slabels, rhos);

	//DBSCAN
	//		clustering_by_DBSCAN(*features, 4.0, 4, cluster_centers, slabels, rhos);

	//OPTICS
	//		clustering_by_optics(*features, 4.0, 4, cluster_centers, slabels, rhos);

	//cluster center
	pcl::PointIndicesPtr center_indices (new pcl::PointIndices);
	center_indices->indices = peaks_indices;

	pcl::PointCloud<MyLasPoint>::Ptr cluster_center (new pcl::PointCloud<MyLasPoint>);
	pcl::ExtractIndices<MyLasPoint> extract;
	extract.setInputCloud (unique_cloud);
	extract.setIndices (center_indices);
	extract.filter (*cluster_center);

	//output 
	pcl::LASWriter las_writer;
	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
	pcl::PointCloud<MyLasPoint>::Ptr out_cloud(new pcl::PointCloud<MyLasPoint>);

	char sCenterName[256], sSegmentsName[256];

	sprintf(sCenterName, "%s_sCenter.las", pOutputName);
	sprintf(sSegmentsName, "%s", pOutputName);

	out_cloud->height = cluster_center->height;
	out_cloud->width = cluster_center->width;
	out_cloud->points.resize(cluster_center->points.size());

	for(int i=0; i<cluster_center->points.size(); i++)
	{
		out_cloud->points[i].x = cluster_center->points[i].x;
		out_cloud->points[i].y = cluster_center->points[i].y;
		out_cloud->points[i].z = cluster_center->points[i].z;
		out_cloud->points[i].classification = slabels[i];
		out_cloud->points[i].intensity = rhos[i];
		//unique_cloud->points[i].label = labels[i];
	}

	if(out_cloud->points.size()>0)
		pcl::toROSMsg(*out_cloud, *outCloud2);	

	las_writer.write (sCenterName, *outCloud2);


	out_cloud->height = unique_cloud->height;
	out_cloud->width = unique_cloud->width;
	out_cloud->points.resize(unique_cloud->points.size());

	for(int i=0; i<unique_cloud->points.size(); i++)
	{
		out_cloud->points[i].x = unique_cloud->points[i].x;
		out_cloud->points[i].y = unique_cloud->points[i].y;
		out_cloud->points[i].z = unique_cloud->points[i].z;
		out_cloud->points[i].classification = slabels[i];
		out_cloud->points[i].intensity = rhos[i];
		//unique_cloud->points[i].label = slabels[i];
	}

	if(out_cloud->points.size()>0)
		pcl::toROSMsg(*out_cloud, *outCloud2);	

	las_writer.write (sSegmentsName, *outCloud2);

	return 0;
}

int segment_slicing_pointcloud(char *pInputName, double slice_interval, double neighborRadius, double dc, double k_percent, int min_reachablepts,
	char *pOutputDir)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read(pInputName, *inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_x = boost::math::iround(incloud->points[0].x);
	double offset_y = boost::math::iround( incloud->points[0].y);
	double offset_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_x) < 10000) offset_x = 0;
	if(fabs(offset_y) < 10000) offset_y = 0;
	if(fabs(offset_z) < 10000) offset_z = 0;

	
	pcl::PointCloud<MyLasPoint>::Ptr unique_cloud (new pcl::PointCloud<MyLasPoint>); 
	remove_duplicate_points(*incloud, *unique_cloud);


	std::vector<uint32_t> global_labels;     //labels for the whole scene
	std::vector<myPointIndices> slice_indices;
	
	
	height_slicing(*unique_cloud, slice_interval, slice_indices);


	pcl::PointCloud<MyLasPoint>::Ptr slice_points (new pcl::PointCloud<MyLasPoint>);
	pcl::ExtractIndices<MyLasPoint> extract;


	int islice = 0;
	for(std::vector<myPointIndices>::iterator itIndices=slice_indices.begin(); itIndices!=slice_indices.end(); ++itIndices, ++islice)
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
		clustering_by_density_peaks_modified(*features, neighborRadius, dc, k_percent, min_reachablepts,
			cluster_centers, slabels, rhos);

		//DBSCAN
		//		clustering_by_DBSCAN(*features, 4.0, 4, cluster_centers, slabels, rhos);

		//OPTICS
		//		clustering_by_optics(*features, 4.0, 4, cluster_centers, slabels, rhos);

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

		sprintf(sCenterName, "%s\\sCenters%d.las", pOutputDir, islice);
		sprintf(sSegmentsName, "%s\\segment_neighs%d.las", pOutputDir, islice);

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

		if(out_cloud->points.size()>0)
			pcl::toROSMsg(*out_cloud, *outCloud2);	

		//		las_writer.write (sCenterName, *outCloud2);


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

		if(out_cloud->points.size()>0)
			pcl::toROSMsg(*out_cloud, *outCloud2);	

		las_writer.write (sSegmentsName, *outCloud2);
	}

	return 0;
}

#include "feature_extract/StatisticalOutlierRemoval.h"
int outlier_removing(char *pInputName, int k_nn, float mulThresh, char *pOutputName)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read(pInputName,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_x = boost::math::iround(incloud->points[0].x);
	double offset_y = boost::math::iround( incloud->points[0].y);
	double offset_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_x) < 10000) offset_x = 0;
	if(fabs(offset_y) < 10000) offset_y = 0;
	if(fabs(offset_z) < 10000) offset_z = 0;


	std::vector<int> filterd_indices;
	std::vector<uint32_t> slabels;	//labels for current slice points 
	std::vector<double> rhos;

	//extract features 
	pcl::PointCloud<pcl::PointXYZ>::Ptr features(new pcl::PointCloud<pcl::PointXYZ>);
	features->width = incloud->width;
	features->height = incloud->height;

	for(int k=0; k < incloud->points.size(); k++)
	{
		pcl::PointXYZ pt;

		pt.x = incloud->points[k].x - offset_x;
		pt.y = incloud->points[k].y - offset_y;
		pt.z = incloud->points[k].z - offset_z;

		features->points.push_back(pt);
	}

	pcl_StatisticalOutlierRemoval (*features, k_nn, mulThresh, filterd_indices, true);

	
	//cluster center
	pcl::PointIndicesPtr removed_indices (new pcl::PointIndices);
	removed_indices->indices = filterd_indices;

	pcl::PointCloud<MyLasPoint>::Ptr removed_pts (new pcl::PointCloud<MyLasPoint>);
	pcl::ExtractIndices<MyLasPoint> extract;
	extract.setInputCloud (incloud);
	extract.setIndices (removed_indices);
	extract.filter (*removed_pts);

	//output 
	pcl::LASWriter las_writer;
	pcl::PCLPointCloud2::Ptr outCloud2(new pcl::PCLPointCloud2);
//	pcl::PointCloud<MyLasPoint>::Ptr out_cloud(new pcl::PointCloud<MyLasPoint>);

// 	out_cloud->height = removed_pts->height;
// 	out_cloud->width = removed_pts->width;
// 	out_cloud->points.resize(removed_pts->points.size());
// 
	for(int i=0; i<filterd_indices.size(); i++)
	{
		incloud->points[filterd_indices[i]].classification = 18;

		//unique_cloud->points[i].label = labels[i];
	}

	if(removed_pts->points.size()>0)
		pcl::toROSMsg(*incloud, *outCloud2);	

	las_writer.write (pOutputName, *outCloud2);

	return 0;
}


bool comp(std::vector<int> left, std::vector<int> right) 
{
	return left.size() > right.size();
}

#include "wrapped_applications.h"

// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/visualization/cloud_viewer.h>
#include "segmentations/urban_partition.h"
#include "3D_model/ssg_voxel_grid_graph_builder.h"


int _tmain(int argc, _TCHAR* argv[])
{
//      eliminate facade points
//  	remove_facadePoints("G:/pointcloud/Dundas_University/CSF_fitered/off-ground points.las", 1.0, 25,
// 		"G:/pointcloud/Dundas_University/CSF_fitered/off-ground points_nonfacade.las");
//  	return 0;

//  using AP to segment whole pointcloud
// 	segment_whole_pointcloud("G:/temp/test1.las",
// 		10.0, 4.0, 0.05, 10, "G:/temp/test1_segwhole.las");
// 	return 0;

//  using pcl statistical outlier removal method, it is not work  
// 	outlier_removing("G:/pointcloud/Dundas_University/ALTM_Strip_Dundas.las", 25, 1.5,
// 		"G:/pointcloud/Dundas_University/ALTM_Strip_Dundas_removalOutliers.las");
// 	return 0;

// 	merge_overlap_points("G:/pointcloud/Dundas_University/ALTM_Strip_Dundas_rectified.las", 
// 		"G:/pointcloud/Dundas_University/LYNX_Strip_Dundas.las", 1.0, 
// 		"G:/pointcloud/Dundas_University/merged.las");
// 	return 0;

	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 

	lasreader.read("G:/temp/test3_air_mob_merged.las",
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_ref_x = boost::math::iround( incloud->points[0].x);
	double offset_ref_y = boost::math::iround( incloud->points[0].y);
	double offset_ref_z = boost::math::iround( incloud->points[0].z);

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

// 	output_voxel_model(*org_pts, 2.0, "G:/temp/test3_airborne_2_voxel.ply", 
// 		offset_ref_x, offset_ref_y, offset_ref_z,
// 		"G:/temp/test3_airborne_2_centriod.las");


	//build_voxel_adjacency_graph(*org_pts, 2.0);
	build_voxel_FC_graph(*org_pts, 2.0);

	return 0;


/*	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read("G:\\pointcloud\\Dundas_University\\CSF_fitered\\off-ground points_subsamples1.las",
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
	std::vector<myPointIndices> slice_indices;
//	height_slicing(*unique_cloud, 3.0, slice_indices);

	pcl::PointCloud<MyLasPoint>::Ptr slice_points (new pcl::PointCloud<MyLasPoint>);
	pcl::ExtractIndices<MyLasPoint> extract;

	
	//Sep 9th, test no slicing
	myPointIndices idx;
	for(int i=0; i<unique_cloud->points.size(); i++)
	{
		idx.push_back(i);
	}
	slice_indices.push_back(idx);

	

	int islice = 0;
	for(std::vector<myPointIndices>::iterator itIndices=slice_indices.begin(); itIndices!=slice_indices.end(); ++itIndices, ++islice)
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
		clustering_by_density_peaks_modified(*features, 10.0, 4.0, 0.05, 4, cluster_centers, slabels, rhos);
		
		//DBSCAN
//		clustering_by_DBSCAN(*features, 4.0, 4, cluster_centers, slabels, rhos);

		//OPTICS
//		clustering_by_optics(*features, 4.0, 4, cluster_centers, slabels, rhos);

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

		sprintf(sCenterName, "G:\\pointcloud\\Dundas_University\\CSF_fitered\\sCenters%d.las", islice);
		sprintf(sSegmentsName, "G:\\pointcloud\\Dundas_University\\CSF_fitered\\segment_neighs%d.las", islice);

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

		if(out_cloud->points.size()>0)
			pcl::toROSMsg(*out_cloud, *outCloud2);	

//		las_writer.write (sCenterName, *outCloud2);


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

		if(out_cloud->points.size()>0)
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

	return 0;*/
}

