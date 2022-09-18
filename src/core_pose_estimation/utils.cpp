/**
 * @file utils.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file for utils. Contains normal computation and .obj to point cloud converter functions
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#include "utils.h"

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d_omp.h>



pcl::PointCloud<pcl::PointNormal>::Ptr utils::objToCloud(std::string obj_file_path)
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileOBJ(obj_file_path, mesh);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	cloud->is_dense = false;
	
	return cloud;
}


pcl::PointCloud<pcl::PointNormal>::Ptr utils::computeCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
																  float radius)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_est;
	
	normal_est.setInputCloud(cloud);
	normal_est.setRadiusSearch(radius);
	normal_est.compute(*normal_cloud);

	// Copy the xyz info from point_cloud and add it to normals as the xyz field in PointNormals estimation is zero
	for(std::size_t i = 0; i<normal_cloud->size(); ++i)
	{
		(*normal_cloud)[i].x = (*cloud)[i].x;
		(*normal_cloud)[i].y = (*cloud)[i].y;
		(*normal_cloud)[i].z = (*cloud)[i].z;
	}

	return normal_cloud;
}