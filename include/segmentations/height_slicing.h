#ifndef  _HEIGHT_SLICING_H_Tony_2046_AUG_19_
#define _HEIGHT_SLICING_H_Tony_2046_AUG_19_

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

typedef std::vector<int> PointIndices; 

template <typename PointT>
int height_slicing(const pcl::PointCloud<PointT> &cloud, double h_interval, std::vector<PointIndices> &slice_indices)
{
	float resolution = 5.0f;

	pcl::octree::OctreePointCloudSearch<PointT> octree (resolution);

	octree.setInputCloud (cloud.makeShared());
	octree.addPointsFromInputCloud ();

	Eigen::Vector3f bbmin, bbmax;

	bbmin[0]=bbmin[1]=bbmin[2]=std::numeric_limits<float>::max();
	bbmax[0]=bbmax[1]=bbmax[2]=std::numeric_limits<float>::min();
	//get boundary
	for(int i=0; i<cloud.points.size(); i++)
	{
		bbmin[0] = (bbmin[0]<cloud.points[i].x) ? bbmin[0] : cloud.points[i].x;
		bbmin[1] = (bbmin[1]<cloud.points[i].y) ? bbmin[1] : cloud.points[i].y;
		bbmin[2] = (bbmin[2]<cloud.points[i].z) ? bbmin[2] : cloud.points[i].z;

		bbmax[0] = (bbmax[0]>cloud.points[i].x) ? bbmax[0] : cloud.points[i].x;
		bbmax[1] = (bbmax[1]>cloud.points[i].y) ? bbmax[1] : cloud.points[i].y;
		bbmax[2] = (bbmax[2]>cloud.points[i].z) ? bbmax[2] : cloud.points[i].z;
	}
//	pcl::getMinMax3D (cloud, bbmin, bbmax);

	Eigen::Vector3f slice_min, slice_max;

	double minz, maxz;
	maxz = bbmax[2];
	minz = bbmin[2];

	slice_max = Eigen::Vector3f (bbmax[0], bbmax[1], bbmax[2]);
	slice_min = Eigen::Vector3f (bbmin[0], bbmin[1], bbmax[2] - h_interval);
	
	while(1)
	{
		std::vector<int> k_indices;
		
		if(slice_min[2] < minz)
		{
			slice_min[2] = minz;
		}
				
		if(octree.boxSearch (slice_min, slice_max, k_indices)>10)
		{
			slice_indices.push_back(k_indices);
		}

		slice_max[2] = slice_min[2];
		slice_min[2] = slice_max[2] - h_interval;

		if(slice_max[2] <= minz)
			break;
	}

	return 0;
}






#endif