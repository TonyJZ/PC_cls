

#include <pcl_las/las_io.h>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/version.hpp>

#include <pcl/PCLPointCloud2.h>

#include <fstream>  // std::ifstream
#include <iostream> // std::cout

//bytes of point record in PCLPointCloud2

pcl::LASReader::LASReader()
{

}

pcl::LASReader::~LASReader()
{
}

int pcl::LASReader::readHeader(const std::string & file_name, pcl::PCLPointCloud2 & cloud,
	Eigen::Vector4f & origin, Eigen::Quaternionf & orientation,
	int & file_version, int & data_type, unsigned int & data_idx, const int offset)
{
	std::ifstream ifs;
	ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);

	liblas::Reader reader(ifs);

	liblas::Header const& header = reader.GetHeader();

	//todo
	file_version = header.GetVersionMajor()*10 + header.GetVersionMinor();

	return 0;
}



/*
*
Base properties of all points regardless of dataformat_id Name
x
y
z
intensity
return_number
number_of_returns
scan_direction
flightline_edge
classification
scan_angle
user_data

For PCL, I am just going to use x,y,z, itensity and colro
*/

int pcl::LASReader::read(const std::string & file_name,
	pcl::PCLPointCloud2 & cloud,
	Eigen::Vector4f & origin,
	Eigen::Quaternionf & orientation,
	int & file_version, const int offset)
{
	std::ifstream ifs;
	ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);

	liblas::Reader reader(ifs);


	unsigned int idx = 0;

	unsigned int nr_points;

	// Setting the is_dense property to true by default
	cloud.is_dense = true;

	{
		pcl::PCLPointField f;
		f.datatype = pcl::PCLPointField::FLOAT64;
		f.count= 1;
		f.name="x";
		cloud.fields.push_back(f);
	}

	{
		pcl::PCLPointField f;
		f.datatype = pcl::PCLPointField::FLOAT64;
		f.count= 1;
		f.name="y";
		f.offset =8;
		cloud.fields.push_back(f);
	}
	{
		pcl::PCLPointField f;
		f.datatype = pcl::PCLPointField::FLOAT64;
		f.count= 1;
		f.name="z";
		f.offset =16;
		cloud.fields.push_back(f);
	}

	{
		pcl::PCLPointField f;
		f.datatype = pcl::PCLPointField::FLOAT32;
		f.count= 1;
		f.name="intensity";
		f.offset =24;
		cloud.fields.push_back(f);
	}

	{
		pcl::PCLPointField f;
		f.datatype = pcl::PCLPointField::UINT32;
		f.count= 1;
		f.name="classification";
		f.offset =28;
		cloud.fields.push_back(f);
	}

	const int point_size = 32;
	cloud.data.resize( reader.GetHeader().GetPointRecordsCount() * point_size);
	cloud.height =1;
	cloud.width = reader.GetHeader().GetPointRecordsCount();
	cloud.point_step =point_size;

	for(uint64_t i=0; reader.ReadNextPoint(); i++)
	{
		liblas::Point const& p = reader.GetPoint();
		*( (double *) ( cloud.data.data() +point_size*i     ) )=  p.GetX();
		*( (double *) ( cloud.data.data() + point_size*i +8 ) )=  p.GetY();
		*( (double *) ( cloud.data.data() + point_size*i +16 ) )=  p.GetZ();
		*( (float *) ( cloud.data.data() + point_size*i +24) ) = p.GetIntensity();
		*( (uint32_t *) ( cloud.data.data() + point_size*i +28) ) = p.GetClassification().GetClass();
	}
	return cloud.width*cloud.height;
}

pcl::LASWriter::LASWriter()
{
}


pcl::LASWriter::~LASWriter()
{
}


int pcl::LASWriter::write(const std::string & file_name, const pcl::PCLPointCloud2 & cloud, 
	const Eigen::Vector4f & origin, const Eigen::Quaternionf & orientation, const bool binary)
{

	unsigned numberOfPoints = cloud.height*cloud.width;
	unsigned p_count = 0;

	if (numberOfPoints == 0)
	{
		//		boost::Warning("[LAS] Cloud is empty!");
		return -1;
	}

	std::ofstream ofs;
	ofs.open(file_name, std::ios::out | std::ios::binary); 
	if (ofs.fail())
	{
		return -1;
	}

	liblas::Header header;
	liblas::Writer writer(ofs, header);

	double offset_x = (uint32_t)*((double*)( cloud.data.data()));
	double offset_y = (uint32_t)*((double*)( cloud.data.data() + 8));
	double offset_z = (uint32_t)*((double*)( cloud.data.data() + 16));

	header.SetOffset(offset_x, offset_y, offset_z);

	double lasScale[3];
	lasScale[0] = 0.01;
	lasScale[1] = 0.01;
	lasScale[2] = 0.01;

	header.SetScale(lasScale[0], lasScale[1], lasScale[2]);

	header.SetPointRecordsCount(numberOfPoints);

	double bbmin[3], bbmax[3];
	bbmin[0] = bbmax[0] = offset_x;
	bbmin[1] = bbmax[1] = offset_y;
	bbmin[2] = bbmax[2] = offset_z;

	header.SetDataFormatId(liblas::ePointFormat0);

	writer.SetHeader(header);
	writer.WriteHeader();

	liblas::Point point(&header);
	liblas::Classification classif = point.GetClassification();

	int point_size = cloud.point_step;
	for(unsigned i=0; i<numberOfPoints; i++)
	{
		double x, y, z;
		uint16_t intensity;
		uint32_t val; 

		x = *( (double *) ( cloud.data.data() +point_size*i     ) );
		y = *( (double *) ( cloud.data.data() + point_size*i +8 ) );
		z = *( (double *) ( cloud.data.data() + point_size*i +16 ) );
		intensity = static_cast<uint16_t>(*( (float *) ( cloud.data.data() + point_size*i +24) ));
		val = *( (uint32_t *) ( cloud.data.data() + point_size*i +28) );

		point.SetCoordinates(x, y, z);
		point.SetIntensity(intensity);

		{
			classif.SetClass(val & 31);		//first 5 bits
			classif.SetSynthetic(val & 32); //6th bit
			classif.SetKeyPoint(val & 64);	//7th bit
			classif.SetWithheld(val & 128);	//8th bit
		}
		point.SetClassification(classif);

		try
		{
			writer.WritePoint(point);
		}
		catch (...)
		{
			//result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
			continue;
		}

		if(bbmin[0] > x) bbmin[0] = x;
		if(bbmax[0] < x) bbmax[0] = x;

		if(bbmin[1] > y) bbmin[1] = y;
		if(bbmax[1] < y) bbmax[1] = y;

		if(bbmin[2] > z) bbmin[2] = z;
		if(bbmax[2] < z) bbmax[2] = z;
		p_count++;
	}

	// enquantize and dequantize bounding box before writing it
	if (bbmax[0] > offset_x)
		bbmax[0] = offset_x + ((int)((bbmax[0]-offset_x)/lasScale[0] + 0.5)) * lasScale[0];
	else
		bbmax[0] = offset_x + ((int)((bbmax[0]-offset_x)/lasScale[0] - 0.5)) * lasScale[0];
	if (bbmin[0] > offset_x)
		bbmin[0] = offset_x + ((int)((bbmin[0]-offset_x)/lasScale[0] + 0.5)) * lasScale[0];
	else
		bbmin[0] = offset_x + ((int)((bbmin[0]-offset_x)/lasScale[0] - 0.5)) * lasScale[0];
	if (bbmax[1] > offset_y)
		bbmax[1] = offset_y + ((int)((bbmax[1]-offset_y)/lasScale[1] + 0.5)) * lasScale[1];
	else
		bbmax[1] = offset_y + ((int)((bbmax[1]-offset_y)/lasScale[1] - 0.5)) * lasScale[1];
	if (bbmin[1] > offset_y)
		bbmin[1] = offset_y + ((int)((bbmin[1]-offset_y)/lasScale[1] + 0.5)) * lasScale[1];
	else
		bbmin[1] = offset_y + ((int)((bbmin[1]-offset_y)/lasScale[1] - 0.5)) * lasScale[1];
	if (bbmax[2] > offset_z)
		bbmax[2] = offset_z + ((int)((bbmax[2]-offset_z)/lasScale[2] + 0.5)) * lasScale[2];
	else
		bbmax[2] = offset_z + ((int)((bbmax[2]-offset_z)/lasScale[2] - 0.5)) * lasScale[2];
	if (bbmin[2] > offset_z)
		bbmin[2] = offset_z + ((int)((bbmin[2]-offset_z)/lasScale[2] + 0.5)) * lasScale[2];
	else
		bbmin[2] = offset_z + ((int)((bbmin[2]-offset_z)/lasScale[2] - 0.5)) * lasScale[2];

	header.SetPointRecordsCount(p_count);
	header.SetMax(bbmax[0], bbmax[1], bbmax[2]);
	header.SetMin(bbmin[0], bbmin[1], bbmin[2]);

	writer.SetHeader(header);
	writer.WriteHeader();

	ofs.close();

	return 0;
}

