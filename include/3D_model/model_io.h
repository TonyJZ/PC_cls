#ifndef _Model_IO_H_Tony_Oct_31_2016_
#define _Model_IO_H_Tony_Oct_31_2016_

#include <vtkCubeSource.h>
#include <vtkSmartPointer.h>
#include <vtkAppendPolyData.h>
#include <pcl/io/vtk_lib_io.h>


vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
	vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
	cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
	return cube->GetOutput();
}


template <typename PointT>
int output_voxel_model(pcl::PointCloud<PointT> &cloud, float vSize, const char *pVoxelFileName, 
	double xOffset=0.0, double yOffset=0.0, double zOffset=0.0,
	const char *pCentroidFileName=NULL)
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
// 		centroid.points[i].x += xOffset;
// 		centroid.points[i].y += yOffset;
// 		centroid.points[i].z += zOffset;

		double x = centroid.points[i].x + xOffset;
		double y = centroid.points[i].y + yOffset;
		double z = centroid.points[i].z + zOffset;

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
	writer->SetFileName (pVoxelFileName);
	writer->Write ();



	if(pCentroidFileName)
	{
		write_LAS(pCentroidFileName, centroid, 
			xOffset, yOffset, zOffset);
	}


	return (0);
}


#endif