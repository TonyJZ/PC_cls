#ifndef _FullyConnected_Voxel_Graph_H_Tony_2016_Dec_6_
#define _FullyConnected_Voxel_Graph_H_Tony_2016_Dec_6_


//#include <Eigen/src/Core/Matrix.h>
#include <pcl/segmentation/boost.h>
#include <boost/unordered_map.hpp>


#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/octree/octree_pointcloud.h>
#include "voxel_container.h"


namespace pcl
{


	namespace UrbanRec
	{

#define _Output_foreground_points_		1
#define _Output_background_points_		2
#define _Output_colored_points_			4

		struct  VoxelCell
		{
			int voxel_type;   //occupied; inner; null
			VoxelContainerPointIndices *voxel_att;
			VoxelContainerPointIndices *father;
		};

		template<typename PointT> 
		//typename VoxelContainerT = VoxelContainerPointIndices
		//typename BranchContainerT = OctreeContainerEmpty,
		//typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> 
		//>
		class VoxelFCGraph : public pcl::PCLBase<PointT>
		{

		public:
			using PCLBase<PointT>::initCompute;
			using PCLBase<PointT>::deinitCompute;
			using PCLBase<PointT>::indices_;
			using PCLBase<PointT>::input_;

			typedef pcl::search::Search <PointT> KdTree;
			typedef typename KdTree::Ptr KdTreePtr;
			//typedef pcl::search::Search<PointT>::Ptr KdTreePtr;

		public:
			typedef boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::directedS > Traits;

			typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS,
				boost::property< boost::vertex_name_t, std::string,
				boost::property< boost::vertex_index_t, long,
				boost::property< boost::vertex_color_t, boost::default_color_type,
				boost::property< boost::vertex_distance_t, long,
				boost::property< boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,
				boost::property< boost::edge_capacity_t, double,
				boost::property< boost::edge_residual_capacity_t, double,
				boost::property< boost::edge_reverse_t, Traits::edge_descriptor > > > > mGraph;

			typedef boost::property_map< mGraph, boost::edge_capacity_t >::type CapacityMap;

			typedef boost::property_map< mGraph, boost::edge_reverse_t>::type ReverseEdgeMap;

			typedef Traits::vertex_descriptor VertexDescriptor;

			typedef boost::graph_traits< mGraph >::edge_descriptor EdgeDescriptor;

			typedef boost::graph_traits< mGraph >::out_edge_iterator OutEdgeIterator;

			typedef boost::graph_traits< mGraph >::vertex_iterator VertexIterator;

			typedef boost::property_map< mGraph, boost::edge_residual_capacity_t >::type ResidualCapacityMap;

			typedef boost::property_map< mGraph, boost::vertex_index_t >::type IndexMap;

			typedef boost::graph_traits< mGraph >::in_edge_iterator InEdgeIterator;


		protected:
			double m_bbmin[3], m_bbmax[3];

			

// 			typedef boost::unordered_map<pcl::octree::OctreeKey, VoxelContainerPointIndices*>   VoxelMap;
// 			VoxelMap m_voxelMap;

			std::vector<VoxelCell> vNeighbourhood_;

			std::vector<pcl::octree::OctreeKey>  vLookupList_;  //voxel查找表，根据vNo找到对应的voxel_key。vNo从0开始顺序对voxel编号；voxel_key为八叉树编号

			uint32_t  num_of_occupied_;
			uint32_t  num_of_inner_;
			uint32_t  num_of_null_;


			//高程阈值
			float height_above_ground_;
			float alpha_h_;	


			uint8_t  output_flag_;

		protected:
			/** \brief The size of a leaf. */
			Eigen::Vector4f leaf_size_;
			/** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */ 
			Eigen::Array4f inverse_leaf_size_;
			/** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
			//Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

			//division num in x, y, z 
			int vNumX_, vNumY_, vNumZ_;
			int vLayerNum_;

			Eigen::MatrixXd m_boundMark;

			//pcl::octree::OctreePointCloud<PointT> m_Octree;

			KdTreePtr search_;

			std::vector <pcl::PointIndices> clusters_;  

			/** \brief Signalizes if the graph is valid. */
			bool graph_is_valid_;

			/** \brief Signalizes if the unary potentials are valid. */
			bool unary_potentials_are_valid_;

			/** \brief Signalizes if the binary potentials are valid. */
			bool binary_potentials_are_valid_;

			/** \brief Stores the weight for every edge that comes from source point. */
			double source_weight_;

			/** \brief Used for comparison of the floating point numbers. */
			double epsilon_;

			/** \brief Stores the sigma coefficient. It is used for finding smooth costs. More information can be found in the article. */
			double inverse_sigma_;

			/////////////////////////////////////////////////////////////
			/////////////////////////graph cut  /////////////////////////
			/** \brief Stores the graph for finding the maximum flow. */
			boost::shared_ptr<mGraph> graph_;

			/** \brief Stores the capacity of every edge in the graph. */
			boost::shared_ptr<CapacityMap> capacity_;

			/** \brief Stores reverse edges for every edge in the graph. */
			boost::shared_ptr<ReverseEdgeMap> reverse_edges_;

			/** \brief Stores the vertices of the graph. */
			std::vector< VertexDescriptor > vertices_;

			/** \brief Stores the information about the edges that were added to the graph. It is used to avoid the duplicate edges. */
			std::vector< std::set<int> > edge_marker_;

			/** \brief Stores the vertex that serves as source. */
			VertexDescriptor source_;

			/** \brief Stores the vertex that serves as sink. */
			VertexDescriptor sink_;

			/** \brief Stores the maximum flow value that was calculated during the segmentation. */
			double max_flow_;

		public:
			VoxelFCGraph (/*const double resolution_arg*/);

			/** \brief Empty deconstructor. */
			virtual ~VoxelFCGraph ();

			virtual void
				setInputCloud (const PointCloudConstPtr &cloud);

			virtual void
				setIndices (const IndicesConstPtr &indices);

			inline void
				setVoxelSize (float lx, float ly, float lz)
			{
				leaf_size_[0] = lx; leaf_size_[1] = ly; leaf_size_[2] = lz;
				// Avoid division errors
				if (leaf_size_[3] == 0)
					leaf_size_[3] = 1;
				// Use multiplications instead of divisions
				inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
			}

			//initialize voxels and set up adjacency
			virtual void compute_FC_Voxels();


			//        void
			//        defineBoundingBox (const double min_x_arg, const double min_y_arg, const double min_z_arg,
			//                           const double max_x_arg, const double max_y_arg, const double max_z_arg);

			/* 0: NULL; 1: occupied; 2: inner;    */
			void mark_voxels();

			//清空voxel lable 
			void labels_clear();

			//assign points to voxel
			void voxel_partition ();

			//提取特征
			void voxel_features_extraction();

			void extract (/*std::vector <pcl::PointIndices>& clusters*/);


			////////////////parameters///////////////////////
			void setDatatermParams(float heightTh, float alphaH);

			
			void setOutputFlag(uint8_t flag)
			{
				output_flag_ = flag;
			};

			int saveSegmentedFile ( char *pSaveDir, char *pSaveName );

			//for debug
			int savePoints(const char *fileName);  
			int saveVoxels(const char *fileName);

		protected:

			//搜索voxel邻域
			int search_Neighbour(pcl::octree::OctreeKey	key_arg, std::vector<pcl::octree::OctreeKey> &k_indices);

			/** \brief This method simply builds the graph that will be used during the segmentation. */
			bool buildGraph ();

			/** \brief Returns unary potential(data cost) for the given point index.
			* In other words it calculates weights for (source, point) and (point, sink) edges.
			* \param[in] point index of the point for which weights will be calculated
			* \param[out] source_weight calculated weight for the (source, point) edge
			* \param[out] sink_weight calculated weight for the (point, sink) edge
			*/
			void
				calculateUnaryPotential (pcl::octree::OctreeKey	key_arg, double& source_weight, double& sink_weight);

			/** \brief This method simply adds the edge from the source point to the target point with a given weight.
			* \param[in] source index of the source point of the edge
			* \param[in] target index of the target point of the edge
			* \param[in] weight weight that will be assigned to the (source, target) edge
			*/
			bool
				addEdge (int source, int target, double weight);

			/** \brief Returns the Pairwise potential(smooth cost) for the given indices of points.
			* In other words it returns weight that must be assigned to the edge from source to target point.
			* \param[in] source index of the source point of the edge
			* \param[in] target index of the target point of the edge
			*/
			double
				calculatePairwisePotential (pcl::octree::OctreeKey source, pcl::octree::OctreeKey target);
			
			/** \brief This method analyzes the residual network and assigns a label to every point in the cloud.
			* \param[in] residual_capacity residual network that was obtained during the segmentation
			*/
			void
				assembleLabels (ResidualCapacityMap& residual_capacity);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr
				getColoredCloud (/* char *pSaveDir, bool saveFile */bool &has_fore);
		};


	}

}

#include "segmentations/impl/voxel_FC_graph.hpp"

#endif