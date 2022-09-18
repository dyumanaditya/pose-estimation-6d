/**
 * @file segmentation.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file for segmentation. Contains plane segmentation, region segmentation and voxel downsampling
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#include "segmentation.h"

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>



pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation::segPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
                                                           float filter_lim_xmin,
                                                           float filter_lim_xmax,
                                                           float filter_lim_ymin,
                                                           float filter_lim_ymax,
                                                           float filter_lim_zmin,
                                                           float filter_lim_zmax,
                                                           float distance_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

    // Passthrough filter on x-axis to get rid of points after the table
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(scene_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(filter_lim_xmin, filter_lim_xmax);
    pass.setFilterLimitsNegative(false);
    pass.filter(*seg_cloud);

    // Passthrough filter on y-axis to get rid of points after the table
    pass.setInputCloud(seg_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(filter_lim_ymin, filter_lim_ymax);
    pass.setFilterLimitsNegative(false);
    pass.filter(*seg_cloud);

    // Passthrough filter on z-axis to get rid of points after the table
    pass.setInputCloud(seg_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_lim_zmin, filter_lim_zmax);
    pass.setFilterLimitsNegative(false);
    pass.filter(*seg_cloud);

    // Segment Plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(seg_cloud);
    seg.segment(*inliers, *coefficients);
	
    // Extract indices
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(seg_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*seg_cloud);
    if (seg_cloud->points.empty())
    {
        std::cerr << "[segmentation::segPlane] Can't find the planar component." << std::endl;
    }
    else
    {
        std::cerr << "[segmentation::segPlane] PointCloud representing the planar component: " << seg_cloud->size () << " data points." << std::endl;
    }

    return seg_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation::segRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, 
															   Eigen::Vector3f center,
															   float delta)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::CropBox<pcl::PointXYZ> boxFilter;

	boxFilter.setMin(Eigen::Vector4f(center[0]-delta, center[1]-delta, center[2]-delta, 1.0));
	boxFilter.setMax(Eigen::Vector4f(center[0]+delta, center[1]+delta, center[2]+delta, 1.0));
	boxFilter.setInputCloud(scene_cloud);
	boxFilter.filter(*seg_cloud);

    return seg_cloud;
}
