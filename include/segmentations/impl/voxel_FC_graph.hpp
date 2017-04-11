#ifndef _FullyConnected_Voxel_Graph_HPP_Tony_2016_Dec_5_
#define _FullyConnected_Voxel_Graph_HPP_Tony_2016_Dec_5_

#include <limits>

#include <Eigen/src/Core/Matrix.h>
#include <pcl/octree/octree_impl.h>
//#include "segmentations/voxel_Fcc_container.h"


namespace pcl
{

	namespace octree
	{

		static size_t hash_value (const OctreeKey& b)
		{
			return boost::hash_value (b.key_);
		}

	}

}

template<typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/>
pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::VoxelFCGraph ():
	inverse_sigma_ (16.0),
	binary_potentials_are_valid_ (false),
	epsilon_ (0.0001),
//	radius_ (16.0),
	unary_potentials_are_valid_ (false),
	source_weight_ (0.8),
	search_ (),
//	number_of_neighbours_ (14),
	graph_is_valid_ (false),
//	foreground_points_ (0),
//	background_points_ (0),
	clusters_ (0),
	graph_ (),
	capacity_ (),
	reverse_edges_ (),
	vertices_ (0),
	edge_marker_ (0),
	source_ (),/////////////////////////////////
	sink_ (),///////////////////////////////////
	max_flow_ (0.0),
	height_above_ground_ (10.0),
	alpha_h_ (1.0)
{
	m_bbmin[0]=m_bbmin[1]=m_bbmin[2]=std::numeric_limits<double>::max();
	m_bbmax[0]=m_bbmax[1]=m_bbmax[2]=std::numeric_limits<double>::min();
	//OctreePointCloud(resolution_arg);
	//m_Octree.setResolution(resolution_arg);
};

template<typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/>
pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::~VoxelFCGraph ()
{
	if (search_ != 0)
		search_.reset ();
	if (graph_ != 0)
		graph_.reset ();
	if (capacity_ != 0)
		capacity_.reset ();
	if (reverse_edges_ != 0)
		reverse_edges_.reset ();

// 	foreground_points_.clear ();
// 	background_points_.clear ();
	clusters_.clear ();
	vertices_.clear ();
	edge_marker_.clear ();

	vNeighbourhood_.clear();
//	m_voxelMap.clear();
};

template <typename PointT/*, typename VoxelContainerT*/> void
pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT*/>::setInputCloud (const PointCloudConstPtr &cloud)
{
	pcl::PCLBase<PointT>::setInputCloud (cloud);
//	input_as_cloud_ = true;
//	graph_components_.clear (); // invalidate connected components
}

template <typename PointT/*, typename VoxelContainerT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT*/>::setIndices (const IndicesConstPtr &indices)
{
	pcl::PCLBase<PointT>::setIndices (indices);
}

/*
template<typename PointT, typename VoxelContainerT, typename BranchContainerT, typename OctreeT> void
pcl::UrbanRec::VoxelFCGraph<PointT, VoxelContainerT, BranchContainerT, OctreeT>::defineBoundingBox (
const double min_x_arg,
const double min_y_arg,
const double min_z_arg,
const double max_x_arg,
const double max_y_arg,
const double max_z_arg)
{
m_bbmin[0]=min_x_arg; m_bbmin[1]=min_y_arg; m_bbmin[2]=min_z_arg;
m_bbmax[0]=max_x_arg; m_bbmax[1]=max_y_arg; m_bbmax[2]=max_z_arg;

m_Octree.defineBoundingBox (min_x_arg, min_y_arg, min_z_arg,
max_x_arg, max_y_arg, max_z_arg);
}*/

template <typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::voxel_partition ()
{
	// Has the input dataset been set already?
	if (!input_)
	{
		PCL_WARN ("[pcl::UrbanRec::VoxelFCGraph] No input dataset given!\n");
		//output.width = output.height = 0;
		//output.points.clear ();
		return;
	}


	Eigen::Vector4f min_p, max_p;
	// Get the minimum and maximum dimensions
	getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

	m_bbmin[0]=min_p[0]; m_bbmin[1]=min_p[1]; m_bbmin[2]=min_p[2];
	m_bbmax[0]=max_p[0]; m_bbmax[1]=max_p[1]; m_bbmax[2]=max_p[2];

	// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

	if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return;
	}

	m_vNumX = dx; m_vNumY = dy; m_vNumZ = dz;
	int vlayerNum = m_vNumX * m_vNumY;

	VoxelCell cell;
	cell.voxel_type = null_voxel;
	cell.voxel_att = NULL;
	vNeighbourhood_.resize(m_vNumX*m_vNumY*m_vNumZ, cell);
	

	m_boundMark = Eigen::MatrixXd::Constant(m_vNumX, m_vNumY, -1.0);
//	m_boundMark.resize(m_vNumX, m_vNumY);

	// Compute the minimum and maximum bounding box values
	min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
	max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
	min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
	max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
	min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
	max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

	// Compute the number of divisions needed along all axis
	div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
	div_b_[3] = 0;

	// Set up the division multiplier
	divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

	
	// First pass: go over all points and insert them into the index_vector vector
	// with calculated idx. Points with the same idx value will contribute to the
	// same point of resulting CloudPoint
	std::vector<VoxelCell>::iterator vIter = vNeighbourhood_.begin();
	num_of_occupied_ = 0;
	for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
	{
		if (!input_->is_dense)
			// Check if the point is invalid
			if (!pcl_isfinite (input_->points[*it].x) || 
				!pcl_isfinite (input_->points[*it].y) || 
				!pcl_isfinite (input_->points[*it].z))
				continue;

		int ijk0 = static_cast<int> (floor (input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
		int ijk1 = static_cast<int> (floor (input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
		int ijk2 = static_cast<int> (floor (input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

		pcl::octree::OctreeKey	key_arg;
		key_arg.x = ijk0; key_arg.y = ijk1; key_arg.z = ijk2;

		if(m_boundMark(ijk0, ijk1) < ijk2)
			m_boundMark(ijk0, ijk1) = ijk2;

		int vId = ijk2*vlayerNum+ijk1*m_vNumX+ijk0;
		

		if((vIter+vId)->voxel_type == null_voxel)
		{
			boost::shared_ptr<VoxelContainerPointIndices> newVoxel(new VoxelContainerPointIndices);
			newVoxel->occupyFlag = occupied_voxel;
			newVoxel->addPointIndex(*it);

			//m_voxelMap.insert(std::make_pair(key_arg, newVoxel.get()));

			(vIter+vId)->voxel_type = occupied_voxel;
			(vIter+vId)->voxel_att = newVoxel.get();

			num_of_occupied_++;
		}
		else
		{
			(vIter+vId)->voxel_att->addPointIndex(*it);
		}

/*		VoxelMap::iterator it_voxel;
		it_voxel=m_voxelMap.find(key_arg);
		if(it_voxel != m_voxelMap.end())
		{//exist voxel
			it_voxel->second->addPointIndex(*it);
		}
		else
		{//add new voxel
			boost::shared_ptr<VoxelContainerPointIndices> newVoxel(new VoxelContainerPointIndices);
			newVoxel->occupyFlag = occupied_voxel;
			newVoxel->addPointIndex(*it);

			m_voxelMap.insert(std::make_pair(key_arg, newVoxel.get()));
		}*/

	}

}

template<typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::mark_voxels()
{
// 	boost::shared_ptr<VoxelContainerPointIndices> nullVoxel(new VoxelContainerPointIndices);
// 	nullVoxel->occupyFlag = null_voxel;

	int vlayerNum = m_vNumX * m_vNumY;

	num_of_null_ = 0;
	num_of_inner_ = 0;
	std::vector<VoxelCell>::iterator vIter = vNeighbourhood_.begin();
	for(int iz=m_vNumZ-1; iz>=0; --iz)
	{
		for(int ix=0; ix<m_vNumX; ++ix)
		{
			for(int iy=0; iy<m_vNumY; ++iy)
			{
				pcl::octree::OctreeKey	key_arg;
				key_arg.x = ix; key_arg.y = iy; key_arg.z = iz;

				int vId = iz*vlayerNum + iy*m_vNumX + ix;

				(vIter+vId)->key_arg = key_arg;

				if(iz > m_boundMark(ix, iy))
				{//null voxel
					//m_voxelMap.insert(std::make_pair(key_arg, nullVoxel.get()));
					(vIter+vId)->voxel_type = null_voxel;
					(vIter+vId)->voxel_att = NULL;

					num_of_null_++;
				}
				else if(iz == m_boundMark(ix, iy))
				{//occupied
					;
				}
				else
				{
					int vFId = m_boundMark(ix, iy)*vlayerNum + iy*m_vNumX + ix;

					if((vIter+vId)->voxel_type == occupied_voxel)
					{
						(vIter+vId)->voxel_type |= inner_voxel;
						//(vIter+vId)->voxel_att->occupyFlag = occupied_voxel | inner_voxel;
						(vIter+vId)->father = (vIter+vFId)->voxel_att;

						m_boundMark(ix, iy) = iz;
						num_of_inner_++;
					}
					else
					{
						(vIter+vId)->voxel_type = inner_voxel;
						(vIter+vId)->voxel_att = NULL;
						(vIter+vId)->father = (vIter+vFId)->voxel_att;

						num_of_inner_++;

// 						boost::shared_ptr<VoxelContainerPointIndices> innerVoxel(new VoxelContainerPointIndices);
// 						innerVoxel->occupyFlag = inner_voxel;
// 						innerVoxel->fatherptr = it_voxel->second;
					}

// 					VoxelMap::iterator it_voxel, it_father;
// 					it_voxel=m_voxelMap.find(key_arg);
// 					it_father = m_voxelMap.find(pcl::octree::OctreeKey(ix, iy, m_boundMark(ix, iy)));
// 					assert(it_father != m_voxelMap.end());
// 
// 					if(it_voxel != m_voxelMap.end())
// 					{//occupied voxel
// 						it_voxel->second->occupyFlag = occupied_voxel | inner_voxel;
// 						it_voxel->second->fatherptr = it_father->second;
// 
// 						//update the top voxel
// 						m_boundMark(ix, iy) = iz;
// 					}
// 					else
// 					{
// 						boost::shared_ptr<VoxelContainerPointIndices> innerVoxel(new VoxelContainerPointIndices);
// 						innerVoxel->occupyFlag = inner_voxel;
// 						innerVoxel->fatherptr = it_voxel->second;
// 
// 						m_voxelMap.insert(std::make_pair(key_arg, innerVoxel.get()));
// 					}
				}
			}
		}
	}

}

template<typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::labels_clear()
{
// 	VoxelMap::iterator it_voxel;
// 	for(it_voxel=m_voxelMap.begin(); it_voxel != m_voxelMap.end(); ++it_voxel)
// 	{
// 		it_voxel->second->label = -1;
// 	}
}


template <typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::compute_FC_Voxels ()
{
	if (!initCompute ())
	{
		//graph = GraphT ();
		deinitCompute ();
		return;
	}

	voxel_partition();
	mark_voxels();
}



template <typename PointT/*, typename VoxelContainerT, typename BranchContainerT, typename OctreeT*/> void
	pcl::UrbanRec::VoxelFCGraph<PointT/*, VoxelContainerT, BranchContainerT, OctreeT*/>::voxel_features_extraction()
{
	
// 	if (search_ == 0)
// 	{
// 		search_ = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
// 		search_->setInputCloud (input_, indices_);
// 	}

	int vlayerNum = m_vNumX * m_vNumY;
	for(int ix=0; ix<m_vNumX; ++ix)
	{
		for(int iy=0; iy<m_vNumY; ++iy)
		{
			for(int iz=m_vNumZ-1; iz>=0; --iz)
			{//自顶向下遍历
				int iTop, iBottom; //用于计算平面投影密度

				pcl::octree::OctreeKey	key_arg;
				key_arg.x = ix; key_arg.y = iy; key_arg.z = iz;

				int vId = iz*vlayerNum + iy*m_vNumX + ix;

				std::vector<VoxelCell>::iterator it_voxel;
				it_voxel = vNeighbourhood_.begin()+vId;
				if(it_voxel == vNeighbourhood_.end())
				{
					PCL_ERROR("[pcl::UrbanRec::VoxelFCGraph] Missing voxel (%d, %d, %d)\n", ix, iy, iz);
					continue;
				}

				float cx, cy, cz;

				cx = m_bbmin[0] + leaf_size_[0]*ix + 0.5*leaf_size_[0];
				cy = m_bbmin[1] + leaf_size_[1]*iy + 0.5*leaf_size_[1];
				cz = m_bbmin[2] + leaf_size_[2]*iz + 0.5*leaf_size_[2];

				if(it_voxel->voxel_att)
					it_voxel->voxel_att->feat.vRefPos = Eigen::Vector3f (cx, cy, cz);

				if(it_voxel->voxel_type == null_voxel)
				{//null voxel
					//it_voxel->second->feat.ptsNum = 0;

					continue;
				}
				else if(it_voxel->voxel_type & occupied_voxel)
				{//occupied and inner&occupied
					//计算特征值和特征向量
					
					Eigen::Vector4f xyz_centroid_;
					EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;
					computeMeanAndCovarianceMatrix (*input_, *(it_voxel->voxel_att->getPointIndices()), covariance_matrix_, xyz_centroid_);

					it_voxel->voxel_att->feat.centroid = xyz_centroid_.head<3>();

					Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix_);

					Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
					Eigen::Matrix3f V = es.pseudoEigenvectors();


					Eigen::Vector3f nz(0,0,1);
					it_voxel->voxel_att->feat.major_value_  = D(0,0);
					it_voxel->voxel_att->feat.middle_value_ = D(1,1);
					it_voxel->voxel_att->feat.minor_value_  = D(2,2);

					it_voxel->voxel_att->feat.major_axis_ = V.row(0);
					it_voxel->voxel_att->feat.middle_axis_ = V.row(1);
					it_voxel->voxel_att->feat.minor_axis_ = V.row(2);

					if (nz.dot (it_voxel->voxel_att->feat.minor_axis_) < 0)
					{//consistently re-oriented
						it_voxel->voxel_att->feat.major_axis_ *= -1;
						it_voxel->voxel_att->feat.minor_axis_ *= -1;
					}
				}
				else
				{//inner and no occupied voxel
					continue;
				}


				/*
				else if(it_voxel->second->occupied == inner_voxel)
				{//inner voxel
					it_voxel->second->feat.ptsNum = 0;

					continue;
				}
				else if(it_voxel->second->occupied == occupied_voxel)
				{
					it_voxel->second->feat.ptsNum = it_voxel->second->getSize();



				}



				if(iz > m_boundMark(ix, iy))
				{//null voxel
					m_voxelMap.insert(std::make_pair(key_arg, nullVoxel.get()));
				}
				else if(iz == m_boundMark(ix, iy))
				{//occupied
					;
				}
				else
				{
					VoxelMap::iterator it_voxel;
					it_voxel=m_voxelMap.find(key_arg);
					if(it_voxel != m_voxelMap.end())
					{//occupied voxel
						//update the top voxel
						m_boundMark(ix, iy) = iz;
					}
					else
					{
						boost::shared_ptr<VoxelContainerPointIndices> innerVoxel(new VoxelContainerPointIndices);
						innerVoxel->occupied = inner_voxel;
						innerVoxel->fatherptr = it_voxel->second;

						m_voxelMap.insert(std::make_pair(key_arg, innerVoxel.get()));
					}
				}*/
			}
		}
	}
}

template <typename PointT> void
	pcl::UrbanRec::VoxelFCGraph<PointT>::extract (std::vector <pcl::PointIndices>& clusters)
{
	clusters.clear ();

	bool segmentation_is_possible = initCompute ();
	if ( !segmentation_is_possible )
	{
		deinitCompute ();
		return;
	}

	if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
	{
		clusters.reserve (clusters_.size ());
		std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));
		deinitCompute ();
		return;
	}

	clusters_.clear ();
	bool success = true;

	if ( !graph_is_valid_ )
	{
		success = buildGraph ();
		if (success == false)
		{
			deinitCompute ();
			return;
		}
		graph_is_valid_ = true;
		unary_potentials_are_valid_ = true;
		binary_potentials_are_valid_ = true;
	}

	if ( !unary_potentials_are_valid_ )
	{
		success = recalculateUnaryPotentials ();
		if (success == false)
		{
			deinitCompute ();
			return;
		}
		unary_potentials_are_valid_ = true;
	}

	if ( !binary_potentials_are_valid_ )
	{
		success = recalculateBinaryPotentials ();
		if (success == false)
		{
			deinitCompute ();
			return;
		}
		binary_potentials_are_valid_ = true;
	}

	//IndexMap index_map = boost::get (boost::vertex_index, *graph_);
	ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

	max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

	assembleLabels (residual_capacity);

	clusters.reserve (clusters_.size ());
	std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));

	deinitCompute ();
}


template <typename PointT> bool
	pcl::UrbanRec::VoxelFCGraph<PointT>::buildGraph ()
{
// 	int number_of_points = static_cast<int> (input_->points.size ());
// 	int number_of_indices = static_cast<int> (indices_->size ());
// 
// 	if (input_->points.size () == 0 || number_of_points == 0 || foreground_points_.empty () == true )
// 		return (false);


	if (search_ == 0)
		search_ = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

	graph_.reset ();
	graph_ = boost::shared_ptr< mGraph > (new mGraph ());

	capacity_.reset ();
	capacity_ = boost::shared_ptr<CapacityMap> (new CapacityMap ());
	*capacity_ = boost::get (boost::edge_capacity, *graph_);

	reverse_edges_.reset ();
	reverse_edges_ = boost::shared_ptr<ReverseEdgeMap> (new ReverseEdgeMap ());
	*reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

	VertexDescriptor vertex_descriptor(0);
	vertices_.clear ();
	vertices_.resize (num_of_occupied_ + 2, vertex_descriptor);

	std::set<int> out_edges_marker;
	edge_marker_.clear ();
	edge_marker_.resize (num_of_occupied_ + 2, out_edges_marker);

	for (int i_point = 0; i_point < num_of_occupied_ + 2; i_point++)
		vertices_[i_point] = boost::add_vertex (*graph_);

	source_ = vertices_[num_of_occupied_];
	sink_ = vertices_[num_of_occupied_ + 1];

	//cor_seed_id.indices.clear();
	//cor_seed_id.indices.resize(number_of_indices, -1);
// 	if(cor_seed_id)
// 	{
// 		delete[] cor_seed_id;
// 		cor_seed_id=NULL;
// 	}

//	cor_seed_id=new int[number_of_indices];

	int vlayerNum = m_vNumX * m_vNumY;
	for(int iz=m_vNumZ-1; iz>=0; --iz)
	{
		for(int ix=0; ix<m_vNumX; ++ix)
		{
			for(int iy=0; iy<m_vNumY; ++iy)
			{
				//int point_index = (*indices_)[i_point];
				int vId = iz*vlayerNum + iy*m_vNumX + ix;

				double source_weight = 0.0;
				double sink_weight = 0.0;
				//	calculateUnaryPotential (point_index, source_weight, sink_weight);
				calculateUnaryPotential (vId, source_weight, sink_weight);
				//	fycalculateUnaryPotential2 (point_index, source_weight, sink_weight);
				addEdge (static_cast<int> (source_), vId, source_weight);
				addEdge (vId, static_cast<int> (sink_), sink_weight);

			}
		}
	}

	std::vector<int> neighbours;
	std::vector<float> distances;
	search_->setInputCloud (input_, indices_);
	for (int i_point = 0; i_point < number_of_indices; i_point++)
	{
		int point_index = (*indices_)[i_point];
		search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
		for (size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++)
		{
			//	  double weight = 1;
			//	  double weight = calculateBinaryPotential (point_index, neighbours[i_nghbr]);
			double weight = calculatePairwisePotential (point_index, neighbours[i_nghbr]);
			addEdge (point_index, neighbours[i_nghbr], weight);
			addEdge (neighbours[i_nghbr], point_index, weight);
		}
		neighbours.clear ();
		distances.clear ();
	}

	return (true);
}

template <typename PointT> void
pcl::UrbanRec::VoxelFCGraph<PointT>::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
  double min_dist_to_foreground = std::numeric_limits<double>::max ();
  //double min_dist_to_background = std::numeric_limits<double>::max ();
  double closest_foreground_point[2];
  closest_foreground_point[0] = closest_foreground_point[1] = 0;
  //double closest_background_point[] = {0.0, 0.0};
  double initial_point[] = {0.0, 0.0};

  initial_point[0] = input_->points[point].x;
  initial_point[1] = input_->points[point].y;

  for (size_t i_point = 0; i_point < foreground_points_.size (); i_point++)
  {
    double dist = 0.0;
    dist += (foreground_points_[i_point].x - initial_point[0]) * (foreground_points_[i_point].x - initial_point[0]);
    dist += (foreground_points_[i_point].y - initial_point[1]) * (foreground_points_[i_point].y - initial_point[1]);
    if (min_dist_to_foreground > dist)
    {
      min_dist_to_foreground = dist;
      closest_foreground_point[0] = foreground_points_[i_point].x;
      closest_foreground_point[1] = foreground_points_[i_point].y;
    }
  }

  sink_weight = pow (min_dist_to_foreground / radius_, 0.5);

  source_weight = source_weight_;
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::UrbanRec::VoxelFCGraph<PointT>::addEdge (int source, int target, double weight)
{
  std::set<int>::iterator iter_out = edge_marker_[source].find (target);
  if ( iter_out != edge_marker_[source].end () )
    return (false);

  EdgeDescriptor edge;
  EdgeDescriptor reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( vertices_[source], vertices_[target], *graph_ );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( vertices_[target], vertices_[source], *graph_ );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);

  (*capacity_)[edge] = weight;
  (*capacity_)[reverse_edge] = 0.0;
  (*reverse_edges_)[edge] = reverse_edge;
  (*reverse_edges_)[reverse_edge] = edge;
  edge_marker_[source].insert (target);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::UrbanRec::VoxelFCGraph<PointT>::calculatePairwisePotential (int source, int target) const
{
  double weight = 0.0;
  double distance = 0.0;
  distance += (input_->points[source].x - input_->points[target].x) * (input_->points[source].x - input_->points[target].x);
  distance += (input_->points[source].y - input_->points[target].y) * (input_->points[source].y - input_->points[target].y);
  distance += (input_->points[source].z - input_->points[target].z) * (input_->points[source].z - input_->points[target].z);
  distance *= inverse_sigma_;
  weight = exp (-distance);

  return (weight);
}

template <typename PointT> void
pcl::UrbanRec::VoxelFCGraph<PointT>::assembleLabels (ResidualCapacityMap& residual_capacity)
{
	std::vector<int> labels;
	labels.resize (input_->points.size (), 0);
	int number_of_indices = static_cast<int> (indices_->size ());
	for (int i_point = 0; i_point < number_of_indices; i_point++)
		labels[(*indices_)[i_point]] = 1;

	clusters_.clear ();

	pcl::PointIndices segment;
	clusters_.resize (2, segment);

	OutEdgeIterator edge_iter, edge_end;
	for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
	{
		if (labels[edge_iter->m_target] == 1)
		{
			if (residual_capacity[*edge_iter] > epsilon_)
				clusters_[1].indices.push_back (static_cast<int> (edge_iter->m_target));
			else
				clusters_[0].indices.push_back (static_cast<int> (edge_iter->m_target));
		}
	}
}

template <typename PointT> void
	pcl::UrbanRec::VoxelFCGraph<PointT>::setDatatermParams(float heightTh, float alphaH)
{
	height_above_ground_ = heightTh;
	alpha_h_ = alphaH;
}

#endif