#ifndef _FACADE_DETECTION_H_Tony_Sep_08_2016_
#define _FACADE_DETECTION_H_Tony_Sep_08_2016_


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>

template <typename PointT>
int estimate_ProjectionDensity_XYPlane(pcl::PointCloud<PointT> &cloud, float radius, std::vector<float> &densities)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cloud2D(new pcl::PointCloud<pcl::PointXY>);
	copyPointCloud(cloud, *cloud2D);


	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud (cloud2D);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	//	float radius = 0.5;

	pcl::PointCloud<pcl::PointXY>::iterator iter;

	iter = cloud2D->points.begin();

	for( ; iter != cloud2D->points.end(); iter++)
	{
		int nn = kdtree.radiusSearch (*iter, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if ( nn > 1 )
		{
			densities.push_back(nn-1);
		}
		else
		{
			densities.push_back(0);
		}

	}

	return 1;
}


template <typename PointT>
int estimate_ProjectionDensity_XYPlane_gaussian(pcl::PointCloud<PointT> &cloud, float radius, std::vector<float> &densities)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cloud2D(new pcl::PointCloud<pcl::PointXY>);
	copyPointCloud(cloud, *cloud2D);


	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud (cloud2D);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	//	float radius = 0.5;
	float radius2 = radius*radius;

	pcl::PointCloud<pcl::PointXY>::iterator iter;
	iter = cloud2D->points.begin();
	for( ; iter != cloud2D->points.end(); iter++)
	{
		int nn = kdtree.radiusSearch (*iter, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		float dens = 0;
		for(int ii=1; ii<pointIdxRadiusSearch.size(); ii++)
		{
			double dis_ij2 = pointRadiusSquaredDistance[ii];
			dens += exp(-dis_ij2/radius2);
		}
		densities.push_back(dens);
	}

	return 1;
}


//convert unorganized point cloud to grid and estimate the projection desity for each grid.
template <typename PointT>
int estimate_ProjectionDensity_horizontal_grid(pcl::PointCloud<PointT> &cloud, float gridsizeX, float gridsizeY,
	pcl::PointCloud<PointT> &grid_cloud)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cloud2D(new pcl::PointCloud<pcl::PointXY>);
	copyPointCloud(cloud, *cloud2D);

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud (cloud2D);

	double radius = std::max(gridsizeX, gridsizeY);
	radius /= 2.0;
	double radius2 = radius*radius;

	float minx, miny, maxx, maxy;

	minx = std::numeric_limits<float>::max();
	miny = std::numeric_limits<float>::max();
	maxx = std::numeric_limits<float>::min();
	maxy = std::numeric_limits<float>::min();

	for(int i = 0; i<cloud2D->points.size(); i++)
	{
		minx = std::min(cloud2D->points[i].x, minx);
		miny = std::min(cloud2D->points[i].y, miny);

		maxx = std::max(cloud2D->points[i].x, maxx);
		maxy = std::max(cloud2D->points[i].y, maxy);
	}
	minx -= gridsizeX/2;
	maxx += gridsizeX/2;
	miny -= gridsizeY/2;
	maxy += gridsizeY/2;

	int rowNum, colNum;

	rowNum = std::ceil((maxy - miny)/gridsizeY);
	colNum = std::ceil((maxx - minx)/gridsizeX);

	grid_cloud.height = rowNum;
	grid_cloud.width  = colNum;
	grid_cloud.points.resize(rowNum*colNum);

	for(int i=0; i<rowNum; i++)
	{
		for(int j=0; j<colNum; j++)
		{
			pcl::PointXY pt;
			pt.x = minx+j*gridsizeX;
			pt.y = maxy-i*gridsizeY;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			int nn = kdtree.radiusSearch (pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
			float dens = 0;
			for(int k=0; k<pointIdxRadiusSearch.size(); k++)
			{
				double dis_ij2 = pointRadiusSquaredDistance[k];
				dens += exp(-dis_ij2/radius2);
			}
			grid_cloud.points[i*colNum+j].x = pt.x;
			grid_cloud.points[i*colNum+j].y = pt.y;
			grid_cloud.points[i*colNum+j].z = dens;
		}
	}

	return 1;
}

#endif