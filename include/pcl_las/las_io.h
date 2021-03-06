/*
 * las_io.h
 *
 *  Created on: May 9, 2012
 *      Author: asher
 */

#ifndef LAS_IO_H_
#define LAS_IO_H_

#include <pcl/pcl_exports.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



struct EIGEN_ALIGN16 MyLasPoint              //定义点类型结构

{

	double x, y, z;                

	uint16_t intensity;

	uint32_t classification;

	uint32_t label;

	double gpstime;

	uint16_t red;
	uint16_t green;
	uint16_t blue;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作

}EIGEN_ALIGN16;// 强制SSE对齐

//}

POINT_CLOUD_REGISTER_POINT_STRUCT(MyLasPoint,// 注册点类型宏

	(double,x,x)

	(double,y,y)

	(double,z,z)

	(uint16_t,intensity,intensity)

	(uint32_t, classification, classification)

	(uint32_t, label, label)

	(double, gpstime, gpstime)

	(uint16_t, red, red)

	(uint16_t, green, green)

	(uint16_t, blue, blue)

	)

//class Header;

namespace pcl{

class PCL_EXPORTS LASReader : public FileReader {

  public:
    LASReader();
    virtual ~LASReader();
    /* Load only the meta information (number of points, their types, etc),
            * and not the points themselves, from a given FILE file. Useful for fast
            * evaluation of the underlying data structure.
            *
            * Returns:
            *  * < 0 (-1) on error
            *  * > 0 on success
            * \param[in] file_name the name of the file to load
            * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
            * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
            * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
            * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
            * \param[out] data_type the type of data (binary data=1, ascii=0, etc)
            * \param[out] data_idx the offset of cloud data within the file
            * \param[in] offset the offset in the file where to expect the true header to begin.
            * One usage example for setting the offset parameter is for reading
            * data from a TAR "archive containing multiple files: TAR files always
            * add a 512 byte header in front of the actual file, so set the offset
            * to the next byte after the header (e.g., 513).
            */
		virtual int 
			readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
						Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
						int &file_version, int &data_type, unsigned int &data_idx, const int offset = 0) ;

		virtual int getBoundingBox(double bbmax[3], double bbmin[3]);


          /** \brief Read a point cloud data from a FILE file and store it into a sensor_msgs/PointCloud2.
            * \param[in] file_name the name of the file containing the actual PointCloud data
            * \param[out] cloud the resultant PointCloud message read from disk
            * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
            * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
            * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
            * \param[in] offset the offset in the file where to expect the true header to begin.
            * One usage example for setting the offset parameter is for reading
            * data from a TAR "archive containing multiple files: TAR files always
            * add a 512 byte header in front of the actual file, so set the offset
            * to the next byte after the header (e.g., 513).
            */
          virtual int
          read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &file_version ,
                const int offset = 0);

  };

  class PCL_EXPORTS LASWriter : FileWriter {

  public:
    LASWriter();
    virtual ~LASWriter();

    /** \brief Save point cloud data to a FILE file containing n-D points
           * \param[in] file_name the output file name
           * \param[in] cloud the point cloud data message
           * \param[in] origin the sensor acquisition origin
           * \param[in] orientation the sensor acquisition orientation
           * \param[in] binary set to true if the file is to be written in a binary
           * FILE format, false (default) for ASCII
           */
         virtual int
         write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                const bool binary = false);

  };

}

#include <pcl/ros/conversions.h>
template <typename PointT>
int write_LAS(const char *pFileName, pcl::PointCloud<PointT> cloud, 
	double xOffset, double yOffset, double zOffset)
{
	pcl::PointCloud<MyLasPoint>::Ptr las_cloud (new pcl::PointCloud<MyLasPoint>);
	for(int i=0; i<cloud.points.size(); i++)
	{
		MyLasPoint pt;
		//pt.classification = cID;
		pt.x = cloud.points[i].x + xOffset;
		pt.y = cloud.points[i].y + yOffset;
		pt.z = cloud.points[i].z + zOffset;
		//pt.intensity = densities[i];
		//unique_cloud->points[i].label = labels[i];

		las_cloud->points.push_back(pt);
	}

	las_cloud->width = las_cloud->points.size();
	las_cloud->height = 1;

	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
	if(las_cloud->points.size()>0)
	{
		pcl::toROSMsg(*las_cloud, *cloud2);
	}

	pcl::LASWriter las_writer;

	return las_writer.write (pFileName, *cloud2);
}


#endif /* LAS_IO_H_ */
