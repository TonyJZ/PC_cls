#ifndef _Wrapped_applications_H_Tony_OCT_19_2016_
#define _Wrapped_applications_H_Tony_OCT_19_2016_


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
#include <pcl/common/angles.h>
#include <pcl/common/geometry.h>

#include <pcl/ros/conversions.h>

#include <pcl_las/las_io.h>
//#include "feature_extract/features_point.h"
// #include "segmentations/duplicate_removing.h"
// #include "segmentations/clustering_density_peaks.h"
// #include "segmentations/height_slicing.h"

#include <pcl/features/normal_3d.h>
#include "alignment/pointcloud_register.h"
#include "feature_extract/edge_detection.h"
#include "feature_extract/geometry_detection_by_jlinkage.h"
#include "geometry/model_fitting.h"
#include "geometry/geo_element.h"
#include "alignment/pointcloud_clipping.h"

//this file is wrapping some completed algorithms

#define Is_Debug	true

int regist_airbore_mobile_pointcloud(char *pAirName, char *pMobName, Eigen::Matrix4f &trans_param, double &orgScore, double &fitScore)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 


	lasreader.read(pAirName,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_ref_x = boost::math::iround(incloud->points[0].x);
	double offset_ref_y = boost::math::iround( incloud->points[0].y);
	double offset_ref_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_ref_x) < 10000) offset_ref_x = 0;
	if(fabs(offset_ref_y) < 10000) offset_ref_y = 0;
	if(fabs(offset_ref_z) < 10000) offset_ref_z = 0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr whole_air_pts(new pcl::PointCloud<pcl::PointXYZ>);
	whole_air_pts->width = incloud->width;
	whole_air_pts->height = incloud->height;

	for(int i=0; i<incloud->points.size(); i++)
	{
		pcl::PointXYZ pt;

		pt.x = incloud->points[i].x - offset_ref_x;
		pt.y = incloud->points[i].y - offset_ref_y;
		pt.z = incloud->points[i].z - offset_ref_z;

		whole_air_pts->points.push_back(pt);
	}

	lasreader.read(pMobName,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr whole_mob_pts(new pcl::PointCloud<pcl::PointXYZ>);
	whole_mob_pts->width = incloud->width;
	whole_mob_pts->height = incloud->height;

	for(int i=0; i<incloud->points.size(); i++)
	{
		pcl::PointXYZ pt;

		pt.x = incloud->points[i].x - offset_ref_x;
		pt.y = incloud->points[i].y - offset_ref_y;
		pt.z = incloud->points[i].z - offset_ref_z;

		whole_mob_pts->points.push_back(pt);
	}

	std::vector<int> clip_indices;
	overlap_clipping(*whole_mob_pts, *whole_air_pts, 3.0, clip_indices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr air_overlap(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud (*whole_air_pts, clip_indices, *air_overlap);

	pcl::PointCloud<pcl::PointXYZ>::Ptr air_coords3D(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> sel_indices;
	pcl::PointCloud<pcl::PointNormal> air_sel_pts;
	pcl::PointCloud<pcl::PointNormal> air_ground_pts;
	int blockSize = 200000;
	float f_num = air_overlap->points.size();
	int nBlocks = std::ceil(f_num/blockSize);

	//sub-block processing
	for(int iBlock=0; iBlock < nBlocks; iBlock++)
	{
		int curBlockSize=blockSize;
		if(iBlock*blockSize + curBlockSize > air_overlap->points.size())
			curBlockSize = air_overlap->points.size()-iBlock*blockSize;

		air_coords3D->points.clear();
		for(int i=0; i<curBlockSize; i++)
		{
			pcl::PointXYZ pt;

			pt.x = air_overlap->points[iBlock*blockSize+i].x;
			pt.y = air_overlap->points[iBlock*blockSize+i].y;
			pt.z = air_overlap->points[iBlock*blockSize+i].z;

			air_coords3D->points.push_back(pt);
		}
		air_coords3D->width = curBlockSize;
		air_coords3D->height = 1;

		//using normal estimation to pick out facade points

		pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals_air (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (/*org_sel_pts.makeShared()*/air_coords3D);
		normal_estimator.setKSearch (15);
		normal_estimator.compute (*normals_air);

		const double pi = 3.1415926535897932384626433832795;
		float cosine_threshold = std::cosf (pi*85.0/180.0);
		float ver[3];
		ver[0] = 0; ver[1] = 0; ver[2] = 1;
		Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (ver));
		sel_indices.clear();
		for(int i=0; i<normals_air->points.size(); i++)
		{

			Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_air->points[i].normal));
			float dot_product = fabsf (nghbr_normal.dot (initial_normal));
			if (dot_product < cosine_threshold)
			{   // select facade points
				pcl::PointNormal ptNor;
				ptNor.x = air_coords3D->points[i].x;
				ptNor.y = air_coords3D->points[i].y;
				ptNor.z = air_coords3D->points[i].z;

				ptNor.normal_x = normals_air->points[i].normal_x;
				ptNor.normal_y = normals_air->points[i].normal_y;
				ptNor.normal_z = normals_air->points[i].normal_z;
				air_sel_pts.push_back(ptNor);
			}

			// select ground points
			double cos_upward = cos(pcl::deg2rad(15.0));
			if(dot_product > cos_upward)
			{
				pcl::PointNormal ptNor;
				ptNor.x = air_coords3D->points[i].x;
				ptNor.y = air_coords3D->points[i].y;
				ptNor.z = air_coords3D->points[i].z;

				ptNor.normal_x = normals_air->points[i].normal_x;
				ptNor.normal_y = normals_air->points[i].normal_y;
				ptNor.normal_z = normals_air->points[i].normal_z;
				air_ground_pts.push_back(ptNor);
			}
		}

	}

	air_sel_pts.width = air_sel_pts.points.size();
	air_sel_pts.height = 1;

	air_ground_pts.width = air_ground_pts.points.size();
	air_ground_pts.height = 1;



	//mobile pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr mob_coords3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal> mob_norm_sel_pts;
	pcl::PointCloud<pcl::PointNormal> mob_ground_pts;

	f_num = whole_mob_pts->points.size();
	nBlocks = std::ceil(f_num/blockSize);

	//sub-block processing
	for(int iBlock=0; iBlock < nBlocks; iBlock++)
	{
		int curBlockSize=blockSize;
		if(iBlock*blockSize + curBlockSize > whole_mob_pts->points.size())
			curBlockSize = whole_mob_pts->points.size()-iBlock*blockSize;

		mob_coords3D->points.clear();
		for(int i=0; i<curBlockSize; i++)
		{
			pcl::PointXYZ pt;

			pt.x = whole_mob_pts->points[iBlock*blockSize+i].x;
			pt.y = whole_mob_pts->points[iBlock*blockSize+i].y;
			pt.z = whole_mob_pts->points[iBlock*blockSize+i].z;

			mob_coords3D->points.push_back(pt);
		}
		mob_coords3D->width = curBlockSize;
		mob_coords3D->height = 1;

		//mobile point cloud contains high noises, need preprocess carefully
		pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals_mob (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (/*org_sel_pts.makeShared()*/mob_coords3D);
		normal_estimator.setKSearch (100);
		normal_estimator.compute (*normals_mob);

		const double pi = 3.1415926535897932384626433832795;
		float cosine_threshold = std::cosf (pi*85.0/180.0);
		float ver[3];
		ver[0] = 0; ver[1] = 0; ver[2] = 1;
		Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (ver));
		sel_indices.clear();

		for(int i=0; i<normals_mob->points.size(); i++)
		{
			Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_mob->points[i].normal));
			float dot_product = fabsf (nghbr_normal.dot (initial_normal));
			if (dot_product < cosine_threshold && mob_coords3D->points[i].z > 62.0)
			{
				sel_indices.push_back(i);

				pcl::PointNormal ptNor;
				ptNor.x = mob_coords3D->points[i].x;
				ptNor.y = mob_coords3D->points[i].y;
				ptNor.z = mob_coords3D->points[i].z;

				ptNor.normal_x = normals_mob->points[i].normal_x;
				ptNor.normal_y = normals_mob->points[i].normal_y;
				ptNor.normal_z = normals_mob->points[i].normal_z;
				mob_norm_sel_pts.push_back(ptNor);
			}

			// select ground points
			double cos_upward = cos(pcl::deg2rad(15.0));
			if(dot_product > cos_upward)
			{
				pcl::PointNormal ptNor;
				ptNor.x = mob_coords3D->points[i].x;
				ptNor.y = mob_coords3D->points[i].y;
				ptNor.z = mob_coords3D->points[i].z;

				ptNor.normal_x = normals_mob->points[i].normal_x;
				ptNor.normal_y = normals_mob->points[i].normal_y;
				ptNor.normal_z = normals_mob->points[i].normal_z;
				mob_ground_pts.push_back(ptNor);
			}
		}
	}

	mob_norm_sel_pts.width = air_sel_pts.points.size();
	mob_norm_sel_pts.height = 1;

	mob_ground_pts.width = mob_ground_pts.points.size();
	mob_ground_pts.height = 1;

	//resample ground points 
	pcl::PointCloud<pcl::PointNormal> re_air_ground_pts;
	pcl::PointCloud<pcl::PointNormal> re_mob_ground_pts;

	voxel_downsampling(air_ground_pts, re_air_ground_pts, 1.0, 1.0, 1.0, 5, 1);
	voxel_downsampling(mob_ground_pts, re_mob_ground_pts, 1.0, 1.0, 1.0, 30, 3);

	air_sel_pts += re_air_ground_pts;
	mob_norm_sel_pts += re_mob_ground_pts;


	if(Is_Debug){
		write_LAS("G:/temp/test_whole_cor_air.las"/*"G:/temp/test3_cor_air.las"*/, air_sel_pts, offset_ref_x, offset_ref_y, offset_ref_z);
		write_LAS("G:/temp/test_whole_cor_mob.las"/*"G:/temp/test3_cor_mob.las"*/, mob_norm_sel_pts, offset_ref_x, offset_ref_y, offset_ref_z);
	}
	pcl::PointCloud<pcl::PointNormal> final_cloud;


	double max_s_dis = 1.0; //the max search distance
	//	simple_ICP_nl (air_sel_pts, mob_norm_sel_pts, final_cloud, max_s_dis, 100, 1e-8,
	//		trans_param, fitScore);

	pcl::CorrespondencesPtr corres_ = Customized_ICP(air_sel_pts, mob_norm_sel_pts, final_cloud, max_s_dis, 100, 1e-8,
		15, 1.3, trans_param, fitScore);


	//	sel_indices.clear();
	double after_mse=0;
	double dis2_th = max_s_dis*max_s_dis;
	std::vector<double> residuals;

	for(int i=0; i<corres_->size(); i++)
	{
		int s_ID = corres_->at(i).index_query;
		int t_ID = corres_->at(i).index_match;

		double dis2 = pcl::geometry::squaredDistance(final_cloud.points[s_ID], mob_norm_sel_pts.points[t_ID]);
		after_mse += dis2;
		residuals.push_back(sqrt(dis2));
	}

	after_mse = sqrt(after_mse/sel_indices.size());

	if(Is_Debug){
		output_residual(/*"G:/temp/register_residuals_test3_after_nl.txt"*/"G:/temp/register_residuals__test_whole_after.txt",
			residuals, after_mse);
	}

	orgScore = 0;
	residuals.clear();
	for(int i=0; i<corres_->size(); i++)
	{
		int s_ID = corres_->at(i).index_query;
		int t_ID = corres_->at(i).index_match;

		double dis2 = pcl::geometry::squaredDistance(air_sel_pts.points[s_ID], mob_norm_sel_pts.points[t_ID]);
		orgScore += dis2;
		residuals.push_back(sqrt(dis2));
	}

	orgScore = sqrt(orgScore/sel_indices.size());

	if(Is_Debug){
		output_residual(/*"G:/temp/register_residuals__test3_before_nl.txt"*/"G:/temp/register_residuals__test_whole_before.txt",
			residuals, orgScore);



		write_LAS("G:/temp/align_whole_cors.las"/*"G:/temp/align_test3_cors_nl.las"*/, final_cloud, offset_ref_x, offset_ref_y, offset_ref_z);
	}

	return 0;
}

int rectify_pointcloud(char *pOriFile, char *pDstName, Eigen::Matrix4f *params)
{
	pcl::PCLPointCloud2::Ptr inCloud2(new pcl::PCLPointCloud2);
	pcl::LASReader lasreader;
	Eigen::Vector4f origin;
	Eigen::Quaternionf rot;
	int fv;
	pcl::PointCloud<MyLasPoint>::Ptr incloud (new pcl::PointCloud<MyLasPoint>); 
	std::vector<double> times;
	std::vector<uint16_t> intensities;


	lasreader.read(pOriFile,
		*inCloud2, incloud->sensor_origin_, incloud->sensor_orientation_, fv);

	pcl::fromPCLPointCloud2 (*inCloud2, *incloud);

	double offset_ref_x = boost::math::iround(incloud->points[0].x);
	double offset_ref_y = boost::math::iround( incloud->points[0].y);
	double offset_ref_z = boost::math::iround( incloud->points[0].z);

	if(fabs(offset_ref_x) < 10000) offset_ref_x = 0;
	if(fabs(offset_ref_y) < 10000) offset_ref_y = 0;
	if(fabs(offset_ref_z) < 10000) offset_ref_z = 0;


	//ref pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr ori_coords3D(new pcl::PointCloud<pcl::PointXYZ>);
	ori_coords3D->width = incloud->width;
	ori_coords3D->height = incloud->height;

	for(int k=0; k < incloud->points.size(); k++)
	{
		pcl::PointXYZ pt;

		pt.x = incloud->points[k].x - offset_ref_x;
		pt.y = incloud->points[k].y - offset_ref_y;
		pt.z = incloud->points[k].z - offset_ref_z;

		ori_coords3D->points.push_back(pt);

		times.push_back(incloud->points[k].gpstime);
		intensities.push_back(incloud->points[k].intensity);
	}

	pcl::PointCloud<pcl::PointXYZ> result_cloud;
	pcl::transformPointCloud (*ori_coords3D, result_cloud, *params);

	pcl::PointCloud<MyLasPoint>::Ptr las_cloud (new pcl::PointCloud<MyLasPoint>);
	for(int i=0; i<result_cloud.points.size(); i++)
	{
		MyLasPoint pt;
		//pt.classification = cID;
		pt.x = result_cloud.points[i].x + offset_ref_x;
		pt.y = result_cloud.points[i].y + offset_ref_y;
		pt.z = result_cloud.points[i].z + offset_ref_z;
		pt.intensity = intensities[i];
		pt.gpstime = times[i];
		//unique_cloud->points[i].label = labels[i];

		las_cloud->points.push_back(pt);
	}

	las_cloud->width = las_cloud->points.size();
	las_cloud->height = 1;

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	if(las_cloud->points.size()>0)
	{
		pcl::toROSMsg(*las_cloud, *cloud);
	}

	pcl::LASWriter las_writer;

	las_writer.write (pDstName, *cloud);

	return 0;
}

int points_registering(char *pOrgFile, char *pRefFile, char *pRectifiedFile)
{
	P_CLS::Transform_params params;
	Eigen::Matrix4f trans_param; 
	double orgScore, fitScore;

	regist_airbore_mobile_pointcloud(pOrgFile,/*"G:/pointcloud/Dundas_University/ALTM_Strip_Dundas.las"*//*"G:/temp/test3_airborne.las"*/
		pRefFile,/*"G:/pointcloud/Dundas_University/LYNX_Strip_Dundas.las"*//*"G:/temp/test3_mobile.las"*/
		trans_param, 
		orgScore,
		fitScore);

	//	trans_param = Eigen::Matrix4f::Identity ();
	if(Is_Debug){
		std::ofstream file("G:/temp/transform_params.txt");
		if (file.is_open())
		{
			file << "Here is the matrix :\n" << trans_param << '\n';
			//file << "m" << '\n' <<  colm(m) << '\n';
		}
		file.close();
	}


	rectify_pointcloud(pOrgFile,/*"G:/pointcloud/Dundas_University/ALTM_Strip_Dundas.las"*/
		pRectifiedFile,/*"G:/pointcloud/Dundas_University/ALTM_Strip_Dundas_rectified.las"*/
		&trans_param);

	return 0;
}








#endif