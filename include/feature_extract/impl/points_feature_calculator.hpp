#ifndef _Point_Feature_Calculator_HPP_Tony_2011_Jan_13_
#define _Point_Feature_Calculator_HPP_Tony_2011_Jan_13_

template<typename PointT>
pcl::PointFeatureCalculator<PointT>::PointFeatureCalculator (/*const double resolution_arg*/)//:m_Octree(resolution_arg)
{

};

template <typename PointT> void
	pcl::PointFeatureCalculator<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
	pcl::PCLBase<PointT>::setInputCloud (cloud);
	//	input_as_cloud_ = true;
	//	graph_components_.clear (); // invalidate connected components
}

template <typename PointT> void
	pcl::PointFeatureCalculator<PointT>::setIndices (const IndicesConstPtr &indices)
{
	pcl::PCLBase<PointT>::setIndices (indices);
}


#endif