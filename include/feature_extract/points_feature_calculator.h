#ifndef _Point_Feature_Calculator_H_Tony_2011_Jan_13_
#define _Point_Feature_Calculator_H_Tony_2011_Jan_13_


#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/octree/octree_pointcloud.h>

namespace pcl
{
	template<typename PointT>

	class PointFeatureCalculator : public pcl::PCLBase<PointT>
	{

		using PCLBase<PointT>::initCompute;
		using PCLBase<PointT>::deinitCompute;
		using PCLBase<PointT>::indices_;
		using PCLBase<PointT>::input_;

	public:
		


	public:
		PointFeatureCalculator (/*const double resolution_arg*/);

		/** \brief Empty deconstructor. */
		virtual ~PointFeatureCalculator ()
		{

		};

		virtual void
			setInputCloud (const PointCloudConstPtr &cloud);

		virtual void
			setIndices (const IndicesConstPtr &indices);

		


	protected:


	};
}

#include "segmentations/impl/voxel_FC_graph.hpp"
#endif