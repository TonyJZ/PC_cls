#ifndef _Point_Cloud_Register_H_Tony_Sep_15_2016_
#define _Point_Cloud_Register_H_Tony_Sep_15_2016_

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

namespace P_CLS 
{
	enum Trans_Type
	{
		Non_Trans = 0,
		Rigid2D = 1,
		Rigid3D = 2
	};


	typedef struct  
	{
		Trans_Type type;
		double coeffs[32];

	}Transform_params;

}



template <typename PointT>
int simple_ICP (pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &tar_cloud, pcl::PointCloud<PointT> &final_cloud,
	double search_dis, int maxIter, double convergent_eps, 
	Eigen::Matrix4f &trans_param, double &fitScore)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	// Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (search_dis);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maxIter);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (convergent_eps);

	icp.setInputCloud(org_cloud.makeShared());
	icp.setInputTarget(tar_cloud.makeShared());
	icp.align(final_cloud);
	
	fitScore = icp.getFitnessScore(search_dis);
	
	trans_param = icp.getFinalTransformation();

	return (0);
}

template <typename PointT>
int simple_ICP_nl (pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &tar_cloud, pcl::PointCloud<PointT> &final_cloud,
	double search_dis, int maxIter, double convergent_eps, 
	Eigen::Matrix4f &trans_param, double &fitScore)
{
	pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;

	// Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (search_dis);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maxIter);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (convergent_eps);

	icp.setInputCloud(org_cloud.makeShared());
	icp.setInputTarget(tar_cloud.makeShared());
	icp.align(final_cloud);

	fitScore = icp.getFitnessScore(search_dis);

	trans_param = icp.getFinalTransformation();

	return (0);
}

bool output_residual(char *pFileName, std::vector<double> &residuals, double rms)
{
	FILE *fp=NULL;

	fp = fopen(pFileName, "wt");
	if(fp==NULL)
		return false;

	fprintf(fp, "point num: %d, rms: %.4f\n", residuals.size(), rms);

	for(int i=0; i<residuals.size(); i++)
	{
		fprintf(fp, "%.4f\n", residuals[i]);
	}

	fclose(fp);
	fp=NULL;
	return true;
}

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointNonLinear_Expanded : public pcl::IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> 
{
public:
	pcl::CorrespondencesPtr getCorrespondencesPtr() 
	{
// 		for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
// 			pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
// 			std::cout << "Index of the source point: " << currentCorrespondence.index_query << std::endl;
// 			std::cout << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
// 			std::cout << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
// 			std::cout << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight << std::endl;
// 		}
		return this->correspondences_;
	}
};

template <typename PointT>
pcl::CorrespondencesPtr Customized_ICP (pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &tar_cloud, 
	pcl::PointCloud<PointT> &final_cloud,
	double search_dis, int maxIter, double convergent_eps, 
	double intersection_angleTh, double medianFactor,
	Eigen::Matrix4f &trans_param, double &fitScore)
{

#define Scalar float

	pcl::PointCloud<PointT>::Ptr src(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr tgt(new pcl::PointCloud<PointT>);

	src = org_cloud.makeShared();
	tgt = tar_cloud.makeShared();

	TransformationEstimationPointToPlane<PointT, PointT, Scalar>::Ptr te (new TransformationEstimationPointToPlane<PointT, PointT, Scalar>);
	//TransformationEstimationLM<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationLM<PointNormal, PointNormal, Scalar>);
	//TransformationEstimationSVD<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationSVD<PointNormal, PointNormal, Scalar>);
	CorrespondenceEstimation<PointT, PointT, Scalar>::Ptr cens (new CorrespondenceEstimation<PointT, PointT, Scalar>);
	//CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>);
	//CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>);
	cens->setInputSource (/*org_cloud.makeShared()*/src);
	cens->setInputTarget (/*tar_cloud.makeShared()*/tgt);
	//cens->setSourceNormals (src);

	CorrespondenceRejectorSurfaceNormal::Ptr cor_rej_sn (new CorrespondenceRejectorSurfaceNormal);
	cor_rej_sn->initializeDataContainer<pcl::PointXYZ, pcl::Normal> ();
// 	cor_rej_sn->setInputSource<PointT> (org_cloud.makeShared());
// 	cor_rej_sn->setInputNormals<PointT, PointT> (org_cloud.makeShared());
// 	cor_rej_sn->setInputTarget<PointT> (tar_cloud.makeShared());
// 	cor_rej_sn->setTargetNormals<PointT, PointT> (tar_cloud.makeShared());
 	cor_rej_sn->setThreshold (cos (deg2rad (intersection_angleTh)));
	
	
	CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new CorrespondenceRejectorMedianDistance);
	cor_rej_med->setMedianFactor(medianFactor);
// 	cor_rej_med->setInputSource<PointT> (org_cloud.makeShared());
// 	cor_rej_med->setInputTarget<PointT> (tar_cloud.makeShared());

	CorrespondenceRejectorSampleConsensus<PointT>::Ptr cor_rej_sac (new CorrespondenceRejectorSampleConsensus<PointT>);
// 	cor_rej_sac->setInputSource (org_cloud.makeShared());
// 	cor_rej_sac->setInputTarget (tar_cloud.makeShared());
	cor_rej_sac->setInlierThreshold (0.005);
	cor_rej_sac->setMaximumIterations (10000);

	CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new CorrespondenceRejectorVarTrimmed);
// 	cor_rej_var->setInputSource<PointT> (org_cloud.makeShared());
// 	cor_rej_var->setInputTarget<PointT> (tar_cloud.makeShared());

	CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new CorrespondenceRejectorTrimmed);

	IterativeClosestPointNonLinear_Expanded<PointT, PointT, Scalar> icp;
	icp.setCorrespondenceEstimation (cens);
	icp.setTransformationEstimation (te);

	icp.addCorrespondenceRejector (cor_rej_sn);
	icp.addCorrespondenceRejector (cor_rej_med);
	icp.addCorrespondenceRejector (cor_rej_var);
	
	//icp.addCorrespondenceRejector (cor_rej_tri);
	//icp.addCorrespondenceRejector (cor_rej_sac);
	icp.setInputSource (/*org_cloud.makeShared()*/src);
	icp.setInputTarget (/*tar_cloud.makeShared()*/tgt);

	icp.setMaxCorrespondenceDistance (search_dis);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maxIter);

	icp.getConvergeCriteria()->setAbsoluteMSE(convergent_eps);

	icp.setTransformationEpsilon (1e-10);
//	PointCloud<PointNormal> output;


	icp.align (final_cloud);

	fitScore = icp.getFitnessScore(search_dis);

	trans_param = icp.getFinalTransformation();

#undef Scalar

	return icp.getCorrespondencesPtr();
}

#endif