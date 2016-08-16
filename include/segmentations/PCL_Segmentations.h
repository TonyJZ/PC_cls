#ifndef _USING_PCL_SEGMENTATION_METHODS_H__ZJ_2016_AUG_8_
#define _USING_PCL_SEGMENTATION_METHODS_H__ZJ_2016_AUG_8_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>


template <typename PointT>
int octree_segmentation (const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_filtered,
	float lx, float ly, float lz)
{
	// Create the filtering object: downsample the dataset using a leaf size of lx * ly * lz
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (lx, ly, lz);
	vg.filter (*cloud_filtered);
//	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
}

//typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

bool enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
		return (true);
	else
		return (false);
}

bool enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
		return (true);
	if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
		return (true);
	return (false);
}

bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	if (squared_distance < 10000)
	{
		if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
			return (true);
		if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
			return (true);
	}
	else
	{
		if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
			return (true);
	}
	return (false);
}

template <typename PointT>
int conditional_euclidean_clustering (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
{
	// Data containers used
//	pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
	pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
	pcl::search::KdTree<PointT>::Ptr search_tree (new pcl::search::KdTree<PointT>);
//	pcl::console::TicToc tt;


	// Set up a Normal Estimation class and merge data in cloud_with_normals
	pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
	pcl::NormalEstimation<PointT, PointTypeFull> ne;
	ne.setInputCloud (cloud_out);
	ne.setSearchMethod (search_tree);
	ne.setRadiusSearch (300.0);
	ne.compute (*cloud_with_normals);

	// Set up a Conditional Euclidean Clustering class
	pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
	cec.setInputCloud (cloud_with_normals);
	cec.setConditionFunction (&customRegionGrowing);
	cec.setClusterTolerance (500.0);
	cec.setMinClusterSize (cloud_with_normals->points.size () / 1000);
	cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
	cec.segment (*clusters);
	cec.getRemovedClusters (small_clusters, large_clusters);
//	std::cerr << ">> Done: " << tt.toc () << " ms\n";

	// Using the intensity channel for lazy visualization of the output
	for (int i = 0; i < small_clusters->size (); ++i)
		for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
			cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
	for (int i = 0; i < large_clusters->size (); ++i)
		for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
			cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
	for (int i = 0; i < clusters->size (); ++i)
	{
		int label = rand () % 8;
		for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
			cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
	}

	// Save the output point cloud
// 	std::cerr << "Saving...\n", tt.tic ();
// 	pcl::io::savePCDFile ("output.pcd", *cloud_out);
// 	std::cerr << ">> Done: " << tt.toc () << " ms\n";

	return (0);

}


template <typename PointT>
int simple_euclidean_clustering (const pcl::PointCloud<PointT> &cloud_in, std::vector<uint32_t> &labels, 
	double dTolerance, int minSize, int maxSize)
{
	labels.resize(cloud_in.points.size());
	labels.assign(cloud_in.points.size(), 0);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (/*0.02*/dTolerance); // 2cm
	ec.setMinClusterSize (/*100*/minSize);
	ec.setMaxClusterSize (/*25000*/maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_in);
	ec.extract (cluster_indices);

	int j = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			labels[*pit] = j;
			//cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
// 		cloud_cluster->width = cloud_cluster->points.size ();
// 		cloud_cluster->height = 1;
// 		cloud_cluster->is_dense = true;
// 
// 		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
// 		std::stringstream ss;
// 		ss << "cloud_cluster_" << j << ".pcd";
// 		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	return (0);
}

#include <pcl/segmentation/region_growing.h>

template <typename PointT>
int simple_region_growing (const pcl::PointCloud<PointT> &cloud_in, std::vector<uint32_t> &labels, 
	int minSize, int maxSize, double smoothTh, double curveTh)
{
	labels.resize(cloud_in.points.size());
	labels.assign(cloud_in.points.size(), 0);

	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

// 	pcl::IndicesPtr indices (new std::vector <int>);
// 	pcl::PassThrough<pcl::PointXYZ> pass;
// 	pass.setInputCloud (cloud);
// 	pass.setFilterFieldName ("z");
// 	pass.setFilterLimits (0.0, 1.0);
// 	pass.filter (*indices);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (minSize);
	reg.setMaxClusterSize (maxSize);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (smoothTh/*3.0 / 180.0 * M_PI*/);
	reg.setCurvatureThreshold (curveTh);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
// 	int counter = 0;
// 	while (counter < clusters[0].indices.size ())
// 	{
// 		std::cout << clusters[0].indices[counter] << ", ";
// 		counter++;
// 		if (counter % 10 == 0)
// 			std::cout << std::endl;
// 	}
// 	std::cout << std::endl;

	int j = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			labels[*pit] = j;
			//cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
// 		cloud_cluster->width = cloud_cluster->points.size ();
// 		cloud_cluster->height = 1;
// 		cloud_cluster->is_dense = true;
// 
// 		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
// 		std::stringstream ss;
// 		ss << "cloud_cluster_" << j << ".pcd";
// 		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	return (0);
}

#include <pcl/segmentation/min_cut_segmentation.h>

template <typename PointT>
int simple_min_cut_segmentation (const pcl::PointCloud<PointT> &cloud_in, std::vector<uint32_t> &labels)
{
	labels.resize(cloud_in.points.size());
	labels.assign(cloud_in.points.size(), 0);

//	pcl::IndicesPtr indices (new std::vector <int>);

	pcl::MinCutSegmentation<PointT> seg;
	seg.setInputCloud (cloud);
//	seg.setIndices (indices);

	pcl::PointCloud<PointT>::Ptr foreground_points(new pcl::PointCloud<PointT> ());
	PointT point;
	point.x = 68.97;
	point.y = -18.55;
	point.z = 0.57;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints (foreground_points);

	seg.setSigma (0.25);
	seg.setRadius (3.0433856);
	seg.setNumberOfNeighbours (14);
	seg.setSourceWeight (0.8);

	std::vector <pcl::PointIndices> clusters;
	seg.extract (clusters);

		int j = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			labels[*pit] = j;
		j++;
	}

	std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

// 	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
// 	pcl::visualization::CloudViewer viewer ("Cluster viewer");
// 	viewer.showCloud(colored_cloud);
// 	while (!viewer.wasStopped ())
// 	{
// 	}

	return (0);
}

template <typename PointT>
int differenceofnormals_segmentation (const pcl::PointCloud<PointT> &cloud_in, std::vector<uint32_t> &labels)
{

	return (0);
}

#include <pcl/segmentation/supervoxel_clustering.h>

template <typename PointT>
int supervoxel_segmentation (const pcl::PointCloud<PointT> &cloud_in, std::vector<uint32_t> &labels)
{
	
	float voxel_resolution = 0.008f;

	float seed_resolution = 0.1f;

	float color_importance = 0.2f;

	float spatial_importance = 0.4f;

	float normal_importance = 1.0f;

	//////////////////////////////  //////////////////////////////
	////// This is how to use supervoxels
	//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
// 	if (disable_transform)
// 		super.setUseSingleCameraTransform (false);
	super.setInputCloud (cloud_in);
	super.setColorImportance (color_importance);
	super.setSpatialImportance (spatial_importance);
	super.setNormalImportance (normal_importance);

	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

//	pcl::console::print_highlight ("Extracting supervoxels!\n");
	super.extract (supervoxel_clusters);

	pcl::PointCloud< PointXYZL >::Ptr labeled_cloud = super.getLabeledCloud ();


	labels.resize(cloud_in.points.size());
	labels.assign(cloud_in.points.size(), 0);

	for(size_t i=0; i<labeled_cloud->points.size(); i++)
	{
		labels[i] = labeled_cloud->points[i].label;
	}

	return (0);
}

#endif