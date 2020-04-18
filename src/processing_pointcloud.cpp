#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "localmap_maker/processing_pointcloud.h"

void points::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(pc);
    vg.setLeafSize(0.05f, 0.05f, 100.0f);
    vg.filter(*pc);
}

void points::extract(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, float x_limit, float y_limit)
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    /*filtering in x range*/
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x_limit, x_limit);
    pass.filter(*pc);
    /*filtering in y range*/
    pass.setInputCloud(pc);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y_limit, y_limit);
    pass.filter(*pc);
}

void points::transform(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, std::string sensor_frame_id, ros::Time stamp)
{
    static tf::TransformListener listener;
    try{
        tf::StampedTransform transform;
        listener.lookupTransform("/base_link", sensor_frame_id, ros::Time(0), transform);
        Eigen::Affine3d affine;
        tf::transformTFToEigen(transform, affine);
        pcl::transformPointCloud(*pc, *pc, affine);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
}
