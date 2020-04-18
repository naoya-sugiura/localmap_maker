#ifndef __SCAN_TO_POINTCLOUD_H
#define __SCAN_TO_POINTCLOUD_H

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

class ScanToPointcloud{
public:
    ScanToPointcloud();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
private:
    laser_geometry::LaserProjection projector_;
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Subscriber scan_sub_;
    tf::TransformListener listener_;
};
#endif //__SCAN_TO_POINTCLOUD_H
