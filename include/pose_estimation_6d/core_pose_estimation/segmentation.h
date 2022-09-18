/**
 * @file segmentation.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Header file for segmentation containing the functions for plane segmentation, region segmentation and voxel downsampling
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#ifndef SEGMENTATION_H
#define SEGMENTATION_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>




/**
 * @brief Namespace containing all the segmentation functions
 * 
 */
namespace segmentation
{
    /** 
     * @brief Segment planar components from a given cloud
     *
     * @param scene_cloud Pointer to point cloud belonging to scene
     * 
     * @param filter_lim_min Minimum limit for passthrough filter
     * @param filter_lim_max Maximum limit for passthrough filter
     * @param distance_threshold Distance threshold for sac segmentation
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Pointer to segmented point cloud 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr segPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, 
                                                 float filter_lim_xmin,
                                                 float filter_lim_xmax,
                                                 float filter_lim_ymin,
                                                 float filter_lim_ymax,
                                                 float filter_lim_zmin,
                                                 float filter_lim_zmax,
                                                 float distance_threshold);

    /** 
     * @brief Segment region around a particular point
     *
     * @param scene_cloud Pointer to point cloud belonging to scene
     * 
     * @param center Point around which region should be segmented
     * @param delta Radius of region to be segmented
     * 
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr Pointer to segmented point cloud 
     */													  
    pcl::PointCloud<pcl::PointXYZ>::Ptr segRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, Eigen::Vector3f center, float delta);


    /**
     * @brief Downsample a point cloud using a voxel grid
     * 
     * @tparam PointT Type of point cloud (either PointNormal or PointXYZ)
     * @param scene_cloud Pointer to point cloud belonging to scene
     * @param leaf Leaf size for voxel grid
     * @return pcl::PointCloud<PointT>::Ptr Pointer to downsampled point cloud 
     */
    template <typename PointT> 
    inline typename pcl::PointCloud<PointT>::Ptr voxelDown(typename pcl::PointCloud<PointT>::Ptr scene_cloud, float leaf)
	{
	   	typename pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud<PointT>);
	    pcl::VoxelGrid<PointT> voxel_grid;
	    voxel_grid.setLeafSize(leaf, leaf, leaf);
	    voxel_grid.setInputCloud(scene_cloud);
	    voxel_grid.filter(*seg_cloud);
	 
	    return seg_cloud;
	}

}; // namespace segmentation


#endif
