// Clustering by fast search and find of density peaks
// Science 27 June 2014:
// Vol. 344 no. 6191 pp. 1492-1496
// DOI: 10.1126/science.1242072
// http://www.sciencemag.org/content/344/6191/1492.full//
//
// original Code Author: Eric Yuan
// Blog: http://eric-yuan.me
// You are FREE to use the following code for ANY purpose.
//
// Modified by Tony 2016.08.10

#ifndef _Clustering_by_Fast_Search_and_Find_of_Density_peaks_H_Tony_2016_Aug_10_
#define _Clustering_by_Fast_Search_and_Find_of_Density_peaks_H_Tony_2016_Aug_10_


#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
//#include <math.h>
//#include <cmath>
#include <limits>
#include <boost/math/special_functions/round.hpp>


#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/geometry.h>


typedef std::vector<int> NeighborIndices;  
typedef std::map<int, NeighborIndices>  NeiborIndiceMap;
typedef NeiborIndiceMap::iterator  it_NeighMap;

typedef std::pair<int, int> PointpairIndice;
typedef std::map<PointpairIndice, double>  DistanceMap;
typedef std::map<std::pair<int, int>, double>::iterator it_DistMap;

// pcl::geometry::distance();

//descending vals's indices
static std::vector<double> *pVals;
bool descending_ValIndices(int idx1, int idx2)
{
	return (*pVals)[idx1] > (*pVals)[idx2];
}

//get cut-off distance
//dc = df(M*percent)
template <typename PointT>
double getdc_original(const pcl::PointCloud<PointT> &cloud, DistanceMap *pDistMap, double percent=0.02)
{
	std::vector<double> p2p_dis;  //all point-to-point distance (points with neighbor constraint)

	std::map<PointpairIndice, bool> neigh_flags;

	//as pDistMap contains dis(i,j) and dis(j,i) , extract the upper triangular matrix
// 	int nSamples = cloud.points.size();
// 	int upperNum = nSamples*(nSamples+1)/2;
// 
// 	bool *flags = new bool[upperNum];
// 	memset(flags, 0, sizeof(bool)*upperNum);

	DistanceMap::iterator iter;
	for(iter = pDistMap->begin(); iter != pDistMap->end(); ++iter)
	{
		int i = iter->first.first;
		int j = iter->first.second;

		if(i>j)
		{
			std::swap(i, j);
		}

// 		int idx = 0;
// 		for(int k=0; k<i; k++)
// 		{
// 			idx += (nSamples-k-1);
// 		}
// 
// 		idx+=j;
		PointpairIndice pair_indice(i, j);

		std::map<PointpairIndice, bool>::iterator it_flag;
		it_flag = neigh_flags.find(pair_indice);
		if(it_flag != neigh_flags.end())
		{
			continue;
		}
		else
		{
			p2p_dis.push_back(iter->second);
			neigh_flags.insert(std::pair<PointpairIndice,bool>(pair_indice,true));
		}

	}

// 	//as pDistMap contains dis(i,j) and dis(j,i) , extract the upper triangular matrix
// 	for(int i=0; i<cloud.points.size(); i++)
// 	{
// 		for(int j=i+1; j<cloud.points.size(); j++)
// 		{
// 			PointpairIndice pair_indice(i, j);
// 			DistanceMap::iterator it;
// 
// 			it = pDistMap->find(pair_indice);
// 			if(it != pDistMap->end())
// 			{
// 				p2p_dis.push_back(it->second);
// 			}
// 		}
// 	}

	// ascending distances
	// using default comparison (operator <):
	std::sort(p2p_dis.begin(), p2p_dis.end());

	int num = p2p_dis.size();  //amount of all neighbored points

	int pos = boost::math::iround(num*percent);

	double dc = p2p_dis[pos];

	neigh_flags.clear();

	return dc;
}


//cutoff kernel
int getLocalDensity_cutoff(NeiborIndiceMap *pNeiMap, DistanceMap *pDistMap, double dc, std::vector<double> &rhos)
{
	int nSamples = pNeiMap->size();

	rhos.assign(nSamples, 0.0);

	int i=0;
	for(it_NeighMap iter = pNeiMap->begin(); iter != pNeiMap->end(); ++iter, i++)
	{
		NeighborIndices *curNIdx = &(iter->second);
		for(NeighborIndices::iterator idxIter = curNIdx->begin(); idxIter != curNIdx->end(); ++idxIter)
		{
			int j = *idxIter;
			PointpairIndice pair_indice(i, j);
			double dis_ij = pDistMap->find(pair_indice)->second;

			//cut-off kernel
			if(dis_ij < dc)
				rhos[i] += 1.0;
		}
	}

	return 0;
}

//gaussian kernel
int getLocalDensity_gaussian(NeiborIndiceMap *pNeiMap, DistanceMap *pDistMap, double dc, std::vector<double> &rhos)
{
	int nSamples = pNeiMap->size();

	rhos.assign(nSamples, 0.0);

	int i=0;
	for(it_NeighMap iter = pNeiMap->begin(); iter != pNeiMap->end(); ++iter, i++)
	{
		NeighborIndices *curNIdx = &(iter->second);
		for(NeighborIndices::iterator idxIter = curNIdx->begin(); idxIter != curNIdx->end(); ++idxIter)
		{
			int j = *idxIter;
			PointpairIndice pair_indice(i, j);
			double dis_ij = pDistMap->find(pair_indice)->second;

			//gaussian kernel
			if(dis_ij < dc)
				rhos[i] += exp(-pow(dis_ij/dc, 2));
		}
	}

	return 0;
}

template <typename PointT>
int getLocalDensity_AverageKnnDistance_Ryan(const pcl::PointCloud<PointT> &cloud, std::vector<double> &rhos)
{
	//M = ratio*N;
	double ratio = 0.015;

	int nSamples = cloud.points.size();

	int M = nSamples * ratio;

	rhos.assign(nSamples, 0.0);

	pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud (&cloud);

	for( pcl::PointCloud<PointT>::iterator iter = cloud.points.begin(), int i=0;
		iter != cloud.points.end(); ++iter, i++)
	{
		std::vector<int> NN_Idx;
		std::vector<float> NN_Distance;
		double sum_dis = 0;

		kdtree.nearestKSearch (*iter, M, NN_Idx, NN_Distance);
		for (int j = 1; i < NN_Idx.size (); ++j)
		{
			sum_dis += NN_Distance[j];
		}

		if(NN_Idx.size() > 1)
			rhos[i] = sum_dis / (NN_Idx.size()-1);
	}

	return 0;
}

//get the margin of each cluster center.
//in this phase, the points are supposed to fully connected
template <typename PointT>
void getDistanceToHigherDensity(const pcl::PointCloud<PointT> &cloud, std::vector<double> &rhos, 
	std::vector<int> &rho_indices, std::vector<double> &deltas, std::vector<int> &nneigh)
{
	int nSamples = cloud.points.size();

	deltas.assign(nSamples, 0.0);
	nneigh.assign(nSamples, -1);

	double mindis, maxdis;
	for(int qj = 1; qj < nSamples; qj++)
	{
		PointT pt_j = cloud.points[rho_indices[qj]];

		mindis = std::numeric_limits<double>::max();
		for(int qi = 0; qi < qj; qi++)
		{
			PointT pt_i = cloud.points[rho_indices[qi]];
			double dis = pcl::geometry::distance(pt_j, pt_i);

			if(dis < mindis)
			{
				mindis = dis;
				nneigh[rho_indices[qj]] = rho_indices[qi];
			}
		}

		deltas[rho_indices[qj]] = mindis;
	}

	maxdis = std::numeric_limits<double>::min();
	for(int qj=1; qj<nSamples; qj++)
	{
		if(maxdis < deltas[rho_indices[qj]])
			maxdis = deltas[rho_indices[qj]];
	}

	deltas[rho_indices[0]] = maxdis;
	nneigh[rho_indices[0]] = 0;

//	return 0;
}



template <typename PointT>
int clustering_by_density_peaks(const pcl::PointCloud<PointT> &cloud, double neighborRadius,
	std::vector<int> &cluster_center_idx, std::vector<uint32_t> &labels)
{
	long start, end;

	int nSamples = cloud.points.size();

	pcl::KdTreeFLANN<PointT>::Ptr kdtree (new pcl::KdTreeFLANN<PointT>);

//	pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud (cloud.makeShared());

	NeiborIndiceMap  indices_map;
	DistanceMap  disMap;

	//find KNN neighbors for each point
	int i=0;
	for( pcl::PointCloud<PointT>::const_iterator iter = cloud.points.begin();
		iter != cloud.points.end(); ++iter, i++)
	{
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		
		NeighborIndices indices_nil;
		indices_nil.clear();
		if ( kdtree->radiusSearch (*iter, neighborRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1 )
		{//pointIdxRadiusSearch[0] is the search point 
			NeighborIndices indices;

			//NeighborIndices *indices = boost::shared_ptr<NeighborIndices> (new NeighborIndices);

			indices.assign(pointIdxRadiusSearch.begin()+1, pointIdxRadiusSearch.end());

			indices_map.insert(std::pair<int,NeighborIndices>(i,indices));

			for (int j = 1; j < pointIdxRadiusSearch.size (); ++j)
			{
				//remove duplicate points previously
				assert(fabs(pointRadiusSquaredDistance[j])>1e-6);

				PointpairIndice p_indice = std::make_pair(i, pointIdxRadiusSearch[j]);

				disMap.insert(std::pair<PointpairIndice, double>(p_indice, pointRadiusSquaredDistance[j]));
			}
		}
		else
		{
			indices_map.insert(std::pair<int,NeighborIndices>(i,indices_nil));
		}
	}

	start = clock();

	/*  1. get cutoff param dc        */
	double dc = getdc_original(cloud, &disMap, 0.02);
	std::vector<double> rhos; //density for each points

	/*    2. get local density        */
	getLocalDensity_cutoff(&indices_map, &disMap, dc, rhos);
	

	/*    3. get margin distance      */
	std::vector<int> rho_indices;
	rho_indices.resize(cloud.points.size());
	for(int i=0; i<cloud.points.size(); ++i)
		rho_indices[i] = i;

	pVals = &rhos;
	std::sort(rho_indices.begin(), rho_indices.end(), descending_ValIndices);   //descending the rho indices

	std::vector<double> deltas;  //minimum distance for local density peaks 
	std::vector<int> nneigh;     //the indice of the peak which has the minimum distance to current point

	getDistanceToHigherDensity(cloud, rhos, rho_indices, deltas, nneigh);

// 	int getDistanceToHigherDensity(const pcl::PointCloud<PointT> &cloud, std::vector<double> &rhos, 
// 		std::vector<int> &rho_indices, std::vector<double> &deltas, std::vector<int> &nneigh)

	/* 4. search peaks  */
	std::vector<double> gamma;   //gamma = rho*delta
	gamma.assign(nSamples, 0);

	double max_gamma = std::numeric_limits<double>::min();
	double min_gamma = std::numeric_limits<double>::max();
	for(int i=0; i<nSamples; ++i)
	{
		gamma[i] = rhos[i]*nneigh[i];
		
		if(max_gamma<gamma[i])	max_gamma = gamma[i];
		if(min_gamma>gamma[i])	min_gamma = gamma[i];
	}

	max_gamma += 1e-6;
	min_gamma -= 1e-6;
	double dinterval = max_gamma - min_gamma;
	for(int i=0; i<nSamples; i++)
	{//normalized  
		gamma[i] = (gamma[i]-min_gamma)/dinterval;
	}


	std::vector<int> gamma_idx;
	gamma_idx.assign(nSamples, 0);
	for(int i=0; i<nSamples; i++)
	{
		gamma_idx[i] = i;
	}

	pVals = &gamma;
	std::sort(gamma_idx.begin(), gamma_idx.end(), descending_ValIndices);

	//how to find peaks?
	int nCluster = nSamples * 0.01;

	std::vector<int> candidates_peaks;
	candidates_peaks.assign(gamma_idx.begin(),gamma_idx.begin()+nCluster);

	cluster_center_idx.assign(candidates_peaks.begin(), candidates_peaks.end());

	/* 5. label points      */
	labels.assign(nSamples, 0);   // 0 for unlabeling point 

	uint32_t la = 1;
	for(int i=0; i<nCluster; i++, la++)
	{//label cluster center
		labels[candidates_peaks[i]] = la;
	}

	for(int i=0; i<nSamples; i++)
	{// label the rest points 
		if(labels[rho_indices[i]] == 0)
		{
			labels[rho_indices[i]] = labels[nneigh[rho_indices[i]]]; //label as the nearest peak
			assert(labels[nneigh[rho_indices[i]]] !=0 );
		}
	}

	// detect outliers
	std::vector<int> halos; // 0: core; 1: halo
	halos.assign(nSamples, 0);

	std::vector<double> rhos_b; //the local density upper bound for each cluster
	rhos_b.assign(nCluster+1, 0);  //rhos_b[0] reserved

	for(int i=0; i<nSamples-1; i++)
	{
		for(int j=i+1; j<nSamples; j++)
		{
			if(labels[i] != labels[j])
			{
				double dis = pcl::geometry::distance(cloud.points[i], cloud.points[j]);

				if(dis < dc) //dc or other cut-off parameter
				{//this is a boundary point 
					double rho_aver = 0.5*(rhos[i] + rhos[j]);
					
					//update cluster bound
					if(rho_aver > rhos_b[labels[i]]) rhos_b[labels[i]] = rho_aver;
					if(rho_aver > rhos_b[labels[j]]) rhos_b[labels[j]] = rho_aver;
				}
			}
		}
	}

	//mark the halo
	for(int i=0; i<nSamples; i++)
	{
		if(rhos[i] < rhos_b[labels[i]])
			halos[i] = 1;
		else
			labels[i] = 0; //outlier
	}

	/////////////////////////////////////////////////////////////
	end = clock();

	std::cout<<"used time: "<<((double)(end - start)) / CLOCKS_PER_SEC<<std::endl;

	return 0;
}



#endif