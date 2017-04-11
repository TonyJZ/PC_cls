#ifndef _URBAN_PARTITION_H_Tony_Oct_24th_2016_
#define _URBAN_PARTITION_H_Tony_Oct_24th_2016_

#include <boost/mpl/map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include "graph/edge_weight_computer.h"
//#include "graph/graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"
#include "graph/voxel_grid_graph_builder.h"
#include "graph/common.h"

#include <algorithm>

//#include "factory/edge_weight_computer_factory.h"
//#include "factory/graph_builder_factory.h"


typedef boost::mpl::map
	<
	boost::mpl::pair<pcl::PointXYZ,          pcl::PointNormal>,
	boost::mpl::pair<pcl::PointNormal,       pcl::PointNormal>,
	boost::mpl::pair<pcl::PointXYZRGB,       pcl::PointXYZRGBNormal>,
	boost::mpl::pair<pcl::PointXYZRGBA,      pcl::PointXYZRGBNormal>,
	boost::mpl::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>,
	boost::mpl::pair<pcl::PointXYZI,         pcl::PointXYZINormal>,
	boost::mpl::pair<pcl::PointXYZINormal,   pcl::PointXYZINormal>
	> PointTypeAssociations;


typedef boost::mpl::at<PointTypeAssociations, pcl::PointXYZ>::type PointWithNormal;

/* Public typedefs related with graph. */
typedef boost::subgraph
	<pcl::graph::point_cloud_graph
	<PointWithNormal,
	boost::vecS,
	boost::undirectedS,
	boost::property<boost::vertex_color_t, uint32_t>,
	boost::property<boost::edge_weight_t, float,
	boost::property<boost::edge_index_t, int> > > >           Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor     VertexId;
typedef boost::graph_traits<Graph>::edge_descriptor       EdgeId;
typedef boost::graph_traits<Graph>::vertex_iterator       VertexIterator;
typedef boost::graph_traits<Graph>::edge_iterator         EdgeIterator;
typedef boost::graph_traits<Graph>::adjacency_iterator    AdjacencyIterator;
typedef boost::shared_ptr<Graph>                                   GraphPtr;
typedef boost::shared_ptr<const Graph>                             GraphConstPtr;
typedef boost::reference_wrapper<Graph>                            GraphRef;

bool descending_graph_vertices(GraphRef &g1, GraphRef &g2)
{
	return boost::num_vertices (g1.get()) > boost::num_vertices (g2.get());
}


template <typename PointT>
int build_voxel_adjacency_graph(pcl::PointCloud<PointT> &cloud, float voxel_resolution_)
{
	using namespace pcl::graph;

	BOOST_MPL_ASSERT ((boost::mpl::has_key<PointTypeAssociations, PointT>));
	BOOST_MPL_ASSERT((boost::is_same<pcl::PointXYZ, PointT>));


	/* Public typedefs for point and point cloud types. */
	// 	typedef PointT                                                       Point;
	// 	typedef pcl::PointCloud<Point>                                       PointCloud;
	// 	typedef typename PointCloud::Ptr                                     PointCloudPtr;
	// 	typedef typename PointCloud::ConstPtr                                PointCloudConstPtr;
	
	// 	typedef pcl::PointCloud<PointWithNormal>                             PointWithNormalCloud;
	// 	typedef typename PointWithNormalCloud::Ptr                           PointWithNormalCloudPtr;


	typedef pcl::PointXYZRGBNormal PointWithNormalT;
	typedef pcl::Normal NormalT;
	typedef pcl::PointCloud<PointWithNormalT> PointWithNormalCloudT;
	typedef pcl::PointCloud<NormalT> NormalCloudT;


// 
// 	typedef pcl::search::Search<PointWithNormal> Search;
// 	typedef typename Search::Ptr SearchPtr;

	

//	factory::EdgeWeightComputerFactory<Graph> wc_factory;
//	factory::GraphBuilderFactory<PointWithNormalT, Graph> gb_factory;

//	auto wc = wc_factory.instantiate (0, "");
	
	typedef pcl::graph::GraphBuilder<PointWithNormalT, Graph> GraphBuilderT;
	typename GraphBuilderT::Ptr gbuilder;
	
	gbuilder.reset (new pcl::graph::VoxelGridGraphBuilder<PointWithNormalT, Graph> (voxel_resolution_));
	
	auto& gb = gbuilder;

	pcl::PointCloud<PointWithNormalT>::Ptr cloud_with_normals(new pcl::PointCloud<PointWithNormalT>);
// 	if (normals->size ())
// 		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
// 	else
	pcl::copyPointCloud (cloud, *cloud_with_normals);
	gb->setInputCloud (cloud_with_normals);

	GraphPtr g (new Graph);
	auto& graph = *g;

	gb->compute (graph);

	std::vector<GraphRef> graph_components;
	pcl::graph::createSubgraphsFromConnectedComponents(graph, graph_components);

	std::sort(graph_components.begin(), graph_components.end(), descending_graph_vertices);   //descending the rho indices

	
	char pSubVoxelName[256], pSubPtsName[256];
	pcl::PointCloud<PointT>::Ptr subgraph_pts(new pcl::PointCloud<PointT>);

	//get corresponding index of point-to-voxel
	std::vector<VertexId> point_to_vertex_map_ = gb->getPointToVertexMap ();
	std::vector<std::vector<int>> all_voxel_pts_indices;
	all_voxel_pts_indices.resize(num_vertices(graph));

	for(size_t pId = 0; pId < point_to_vertex_map_.size(); ++pId)
	{
		VertexId vId = point_to_vertex_map_[pId]; 
		all_voxel_pts_indices[vId].push_back(pId);
	}

	//find connected voxels
	for(size_t i=0; i<graph_components.size(); ++i)
	{
		std::vector<VertexId> c_voxel_indices;
		std::vector<int> c_pt_indices;

		for(size_t j=0; j<graph_components[i].get().m_global_vertex.size(); ++j)
		{
			VertexId  vId = graph_components[i].get().m_global_vertex[j];
			c_voxel_indices.push_back(vId);

			c_pt_indices.insert(c_pt_indices.end(),all_voxel_pts_indices[vId].begin(),all_voxel_pts_indices[vId].end());
		}

		pcl::copyPointCloud(cloud, c_pt_indices, *subgraph_pts);

		sprintf(pSubVoxelName, "G:/temp/graph/graph_%d.ply", i+1);
		sprintf(pSubPtsName, "G:/temp/graph/graph_%d.las", i+1);

		output_voxel_model(*subgraph_pts, voxel_resolution_, pSubVoxelName);
		write_LAS(pSubPtsName, *subgraph_pts, 0, 0, 0);

	}
	return 0;
}

#include "segmentations/voxel_FC_graph.h"

template <typename PointT>
int build_voxel_FC_graph(pcl::PointCloud<PointT> &cloud, float voxel_resolution_)
{
//	using namespace pcl::UrbanRec;

	BOOST_MPL_ASSERT ((boost::mpl::has_key<PointTypeAssociations, PointT>));
	BOOST_MPL_ASSERT((boost::is_same<pcl::PointXYZ, PointT>));

	pcl::UrbanRec::VoxelFCGraph<PointT> vGraph;

	vGraph.setInputCloud(cloud.makeShared());
	vGraph.setVoxelSize(voxel_resolution_, voxel_resolution_,voxel_resolution_);

	vGraph.compute_FC_Voxels();


	return 0;
}

#endif