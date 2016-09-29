#ifndef _Geometry_DETECTION_By_JLinkage_H_Tony_Sep_21_2016_
#define _Geometry_DETECTION_By_JLinkage_H_Tony_Sep_21_2016_

#include <pcl/point_cloud.h>
#include "JLinkage/Include/RandomSampler.h"
#include "JLinkage/Include/PrimitiveFunctions.h"
#include "JLinkage/Include/JLinkage.h"

#include "utilities/uti_forJlinkage.h"
#include "utilities/updator.h"


//modelType: 2 for line, 3 for plane
template <typename PointT>
int geometry_detect_by_JLinkage(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int>> &clusters, int modelType,
	float mInlierThreshold, int nDNeigh=10, float nPClose=0.8, float nPFar=0.2,
	int nSampleModels = 5000)
{
	// Function pointers
	std::vector<float>  *(*mGetFunction)(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts);
	float (*mDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt);

	unsigned int nPtDimension;
	unsigned int nMSS; // number of Minimun sample set
	
	
	if(modelType == 2)
	{
		nPtDimension = 2;
		nMSS = 2;
		mGetFunction = GetFunction_Line;
		mDistanceFunction = DistanceFunction_Line;
	}
	else if(modelType == 3)
	{
		nPtDimension = 3;
		nMSS = 3;
		mGetFunction = GetFunction_Plane;
		mDistanceFunction = DistanceFunction_Plane;
	}
	
	std::vector<std::vector<float> *> mDataPoints;
	transformPointCloud_JLData(cloud, nPtDimension, mDataPoints);

	//step 1:  generate models
	RandomSampler mRandomSampler(mGetFunction, mDistanceFunction, 
		nPtDimension, nMSS, cloud.points.size(),true);

	printf("Inizialing Data \n");
	printf("\t Loading Points... \n");
		
	mRandomSampler.SetPoints(&mDataPoints);


	nPClose = 1.0;
	nPFar = 1.0;
	mRandomSampler.SetNFSamplingTypeNN(nDNeigh, nPClose, nPFar, true);

	Updator::InitializeWaitbar("Generating Hypotesis");
	std::vector<std::vector<float> *> *mModels = 
		mRandomSampler.GetNSample(nSampleModels, 0, NULL, &(Updator::UpdateWaitbar));
	Updator::CloseWaitbar();


	// step 2: aggregate similar models
	printf("Initializing Data... \n");
	// Compute the jLinkage Clusterization
	JLinkage mJLinkage(mDistanceFunction, mInlierThreshold, 
		(unsigned int)mModels->size(), true, 
		(unsigned int)(mDataPoints[0])->size());

	std::vector<const sPtLnk *> mPointMap(mDataPoints.size());	

	std::list<sClLnk *> mClustersList;

	unsigned int counter = 0;
	Updator::InitializeWaitbar("Loading Models ");
	for(unsigned int nModelCount = 0; nModelCount < mModels->size(); nModelCount++)
	{
		mJLinkage.AddModel(((*mModels)[nModelCount]));
		++counter;
		Updator::UpdateWaitbar((float)counter/(float)mModels->size());
	}
	Updator::CloseWaitbar();

	counter = 0;
	Updator::InitializeWaitbar("Loading Points ");
	for(std::vector<std::vector<float> *>::iterator iterPts = mDataPoints.begin();
		iterPts != mDataPoints.end(); ++iterPts )
	{
		sPtLnk *tmp = mJLinkage.AddPoint(*iterPts);
		tmp->ptID = counter;
		mPointMap[counter] = tmp;
		++counter;
		Updator::UpdateWaitbar((float)counter/(float)mDataPoints.size());
	}
	Updator::CloseWaitbar();

	Updator::InitializeWaitbar("J-Clusterizing ");
	mClustersList = mJLinkage.DoJLClusterization(Updator::UpdateWaitbar);
	Updator::CloseWaitbar();

	// Write output
	// 	plhs[0] = mxCreateDoubleMatrix(1,mDataPoints->size(), mxREAL);
	// 	double *mTempUintPointer = (double *)mxGetPr(plhs[0]);

	

	unsigned int counterCl = 0;
	clusters.resize(mClustersList.size());
	for(std::list<sClLnk *>::iterator iterCl = mClustersList.begin(); iterCl != mClustersList.end(); ++iterCl)
	{
		unsigned int cnt=0;
		for(std::list<sPtLnk *>::iterator iterPt = (*iterCl)->mBelongingPts.begin(); 
			iterPt != (*iterCl)->mBelongingPts.end(); ++iterPt)
		{
			clusters[counterCl].push_back((*iterPt)->ptID);
		}
		++counterCl;
		
	}

	//release resource
	for(unsigned int i=0; i < mModels->size(); ++i)
		delete (*mModels)[i];
	delete mModels;

	release_JLData(mDataPoints);

	return 1;
}



#endif