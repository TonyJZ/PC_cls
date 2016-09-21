#ifndef _Statistical_Outlier_Removal_H_Tony_Sep_13_2016_
#define _Statistical_Outlier_Removal_H_Tony_Sep_13_2016_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

//this algorithm is not robust and effect for complex unban scene
//it uses a global distance that can't remove outliers completely or over-remove lots of inliers
template <typename PointT>
int pcl_StatisticalOutlierRemoval (const pcl::PointCloud<PointT> &cloud, int k_nn, float mulThresh, 
	std::vector<int> &filterd_indices, bool native=false)
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud.makeShared());
	sor.setMeanK (k_nn);
	sor.setStddevMulThresh (mulThresh);
	sor.setNegative(native);
	sor.filter(filterd_indices);

	return (0);
}





#endif