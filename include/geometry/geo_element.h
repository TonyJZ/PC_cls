#ifndef _Geometry_Element_Definition_H_Tony_Sep_27_2016_
#define _Geometry_Element_Definition_H_Tony_Sep_27_2016_


#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

namespace P_CLS
{
//	template <typename PointT>
	typedef struct
	{
//		pcl::PointCloud<PointT> *cloud;
		std::vector<int> indices;
		pcl::ModelCoefficients coef;

	}GeoElem;
}





#endif