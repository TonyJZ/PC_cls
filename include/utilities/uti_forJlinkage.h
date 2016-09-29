#ifndef _Util_For_JLinkage_H_Tony_Sep_21_2016_
#define _Util_For_JLinkage_H_Tony_Sep_21_2016_

#include <pcl/point_cloud.h>
#include <vector>

template <typename PointT>
void transformPointCloud_JLData(pcl::PointCloud<PointT> &cloud, int dim, std::vector<std::vector<float> *>& JLData)
{
	JLData.resize(cloud.points.size());

	for(unsigned int i=0; i<JLData.size(); i++)
	{
		std::vector<float>* p=NULL;
		switch (dim)
		{
		case 2:
			p = new std::vector<float>(2);
			(*p)[0]=cloud.points[i].x;
			(*p)[1]=cloud.points[i].y;
			break;
		case 3:
			p = new std::vector<float>(3);
			(*p)[0]=cloud.points[i].x;
			(*p)[1]=cloud.points[i].y;
			(*p)[2]=cloud.points[i].z;
			break;
		}

		JLData[i] = p;
	}

}

void release_JLData(std::vector<std::vector<float> *>& JLData)
{
	for(unsigned int i=0; i<JLData.size(); i++)
	{
		delete JLData[i];
		JLData[i] = NULL;
	}

	JLData.clear();
}


#endif