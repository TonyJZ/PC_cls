#ifndef _URBAN_PARTITION_H_Tony_Oct_24th_2016_
#define _URBAN_PARTITION_H_Tony_Oct_24th_2016_

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>

template <typename PointT>
int get_OBB (pcl::PointCloud<PointT> &cloud, float vx, float vy, float vz)
{
	pcl::VoxelGrid<PointT> voxels;
	voxels.setInputCloud (cloud.makeShared());
	voxels.setLeafSize (vx, vy, vz);

	pcl::PointCloud<PointT> centroid;
	voxels.filter (centroid);

	pcl::PointCloud<PointT>::Ptr voxels_bb(new pcl::PointCloud<PointT>);

	for(int i=0; i<centroid.points.size(); i++)
	{
		PointT pt;
		pt.x = centroid.points[i].x - vx;
		pt.y = centroid.points[i].y - vy;
		pt.z = centroid.points[i].z - vz;
		voxels_bb->points.push_back(pt);

		pt.x = centroid.points[i].x + vx;
		pt.y = centroid.points[i].y + vy;
		pt.z = centroid.points[i].z + vz;
		voxels_bb->points.push_back(pt);
	}
	

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud (voxels_bb);
	feature_extractor.compute ();

// 	std::vector <float> moment_of_inertia;
// 	std::vector <float> eccentricity;
// 	pcl::PointXYZ min_point_AABB;
// 	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;

// 	float major_value, middle_value, minor_value;
// 	Eigen::Vector3f major_vector, middle_vector, minor_vector;
// 	Eigen::Vector3f mass_center;

// 	feature_extractor.getMomentOfInertia (moment_of_inertia);
// 	feature_extractor.getEccentricity (eccentricity);
// 	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
// 	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
// 	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
// 	feature_extractor.getMassCenter (mass_center);

	return (0);
}

#include <vtkCubeSource.h>
#include <vtkSmartPointer.h>
#include <vtkAppendPolyData.h>
vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
	vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
	cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
	return cube->GetOutput();
}

#include <pcl/io/vtk_lib_io.h>
template <typename PointT>
int output_voxel_model(pcl::PointCloud<PointT> &cloud, float vSize, const char *pFileName)
{
	pcl::VoxelGrid<PointT> voxels;
	voxels.setInputCloud (cloud.makeShared());
	voxels.setLeafSize (vSize, vSize, vSize);

	pcl::PointCloud<PointT> centroid;
	voxels.filter (centroid);

	vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New();
	size_t i;
	double s = vSize / 2.0;
	for (i = 0; i < centroid.points.size(); i++)
	{

		double x = centroid.points[i].x;
		double y = centroid.points[i].y;
		double z = centroid.points[i].z;

#if VTK_MAJOR_VERSION < 6
		treeWireframe->AddInput(GetCuboid(x - s, x + s, y - s, y + s, z - s, z + s));
#else
		treeWireframe->AddInputData (GetCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
#endif
	}

	vtkSmartPointer<vtkPolyData> polydata;

	polydata = treeWireframe->GetOutput();

	// Convert to PLY and save
	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New ();
#if VTK_MAJOR_VERSION < 6
	writer->SetInput (polydata);
#else
	writer->SetInputData (polydata);
#endif
	writer->SetArrayName ("Colors");
	writer->SetFileTypeToASCII ();
	//writer->SetFileTypeToBinary();
	writer->SetFileName (pFileName);
	writer->Write ();

	return (0);
}



#endif