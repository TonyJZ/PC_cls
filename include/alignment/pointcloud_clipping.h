#ifndef _PointCloud_Clipping_H_Tony_Oct_17th_2016_
#define _PointCloud_Clipping_H_Tony_Oct_17th_2016_

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>


template <typename PointT>
int overlap_clipping(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &tar_cloud, double radius, std::vector<int> &clipping_indices)
{
	std::vector<bool> flags;
	int num_of_tar = tar_cloud.points.size();

	flags.resize(num_of_tar, true);

	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(tar_cloud.makeShared());

	for(int i=0; i<ref_cloud.points.size(); i++)
	{
		std::vector<int>  	k_indices;
		std::vector<float>  k_sqr_distances;
		tree->radiusSearch(ref_cloud, i, radius, k_indices, k_sqr_distances);

		for(int j=0; j<k_indices.size(); j++)
		{
			if(flags[k_indices[j]])
			{
				clipping_indices.push_back(k_indices[j]);
				flags[k_indices[j]] = false;
			}
		}
	}

	return 0;
}

template <typename PointT>
int voxel_downsampling(pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &sampled_cloud,
	float voxel_x, float voxel_y, float voxel_z, int minVoxel, int nVoxelSample)
{
	pcl::VoxelGrid<PointT> voxels;
	voxels.setInputCloud (org_cloud.makeShared());
	voxels.setLeafSize (voxel_x, voxel_y, voxel_z);
	
	pcl::PointCloud<PointT> centroid;
	voxels.filter (centroid);

	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(org_cloud.makeShared());

	float radius = std::max(voxel_x, voxel_y);
	radius = std::max(radius, voxel_z);

	if(nVoxelSample > minVoxel)
		nVoxelSample = minVoxel;

	sampled_cloud.points.clear();

	for(int i=0; i<centroid.points.size(); i++)
	{
		std::vector<int>  	k_indices;
		std::vector<float>  k_sqr_distances;
		tree->radiusSearch(centroid, i, radius, k_indices, k_sqr_distances);

		if(k_indices.size() < minVoxel)
			continue;


		for(int j=0; j<nVoxelSample; j++)
		{
			sampled_cloud.points.push_back(org_cloud.points[k_indices[j]]);
		}
	}

	sampled_cloud.width = sampled_cloud.points.size();
	sampled_cloud.height = 1;
	return 0;
}


#endif