#include "localmap_maker/scan_to_pointcloud.h"

ScanToPointcloud::ScanToPointcloud()
{
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points/scan", 1);
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ScanToPointcloud::scanCallback, this);
    listener_.setExtrapolationLimit(ros::Duration(0.1));
}

void ScanToPointcloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud;
    try{
        projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, listener_);
        cloud_pub_.publish(cloud);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_pointcloud");
    ScanToPointcloud stp;
    ros::spin();
    return 0;
}
