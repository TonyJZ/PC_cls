#ifndef _PointCloud_Partition_HPP_Create_by_Tony_2017_April_17_
#define _PointCloud_Partition_HPP_Create_by_Tony_2017_April_17_

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

template <typename PointT>
int pointcloud_partition_quadtree(pcl::PointCloud<PointT> &cloud, float blocksize, const char* pOutDir)
{
	float inverse_leaf_size_ = 1.0 / blocksize;

	Eigen::Vector4f min_p, max_p;
	// Get the minimum and maximum dimensions
	getMinMax3D(cloud, min_p, max_p);


		// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_)+1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_)+1;
//	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

	if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pointcloud_partition_quadtree] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return 0;
	}

	int64_t rows, cols; 
	cols = dx; rows = dy;

	std::vector<int> lookup;
	lookup.resize(rows*cols, -1);

// 	Eigen::Vector4i min_b_, max_b_;
// 	// Compute the minimum and maximum bounding box values
// 	min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_));
// 	max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_));
// 	min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_));
// 	max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_));
// 	min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_));
// 	max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_));

		
	// First pass: go over all points and insert them into the index_vector vector
	// with calculated idx. Points with the same idx value will contribute to the
	// same point of resulting CloudPoint
	std::vector<std::vector<int>> indices;
	int num_of_occupied_ = 0;
	for (int i=0; i<cloud.size(); ++i)
	{
		if (!cloud.is_dense)
			// Check if the point is invalid
			if (!pcl_isfinite (cloud.points[i].x) || 
				!pcl_isfinite (cloud.points[i].y) || 
				!pcl_isfinite (cloud.points[i].z))
				continue;

		int ijk0 = static_cast<int> (floor ((cloud.points[i].x - min_p[0])* inverse_leaf_size_));
		int ijk1 = static_cast<int> (floor ((cloud.points[i].y - min_p[1])* inverse_leaf_size_));
		//int ijk2 = static_cast<int> (floor (cloud.points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

		int id = lookup[ijk1*cols+ijk0];
		if(id==-1)
		{
			std::vector<int> cell;
			cell.push_back(i);
			indices.push_back(cell);
			lookup[ijk1*cols+ijk0] = num_of_occupied_;
			num_of_occupied_++;
		}
		else
		{
			indices[id].push_back(i);
		}
	}

	std::string dir = pOutDir+'/';
	int cell_num = indices.size();
	pcl::PCDWriter writer;
	char block_name[1024];
	for(int i=0; i<cell_num; ++i)
	{

		sprintf(block_name,"%s\\%03d_blk.pcd", pOutDir, i );
		writer.write<PointT>(block_name, cloud, indices[i], true);
	}

	return 1;
}



#endif
