#include <pcl_ros/point_cloud.h>
#include "localmap_maker/scan_points_to_grid.h"
#include "localmap_maker/processing_pointcloud.h"

ScanPointsToGrid::ScanPointsToGrid()
    : cost_(100)
{
    cloud_sub_ = nh_.subscribe("/points/scan", 1, &ScanPointsToGrid::scanCallback, this);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid/scan", 1);
}

ScanPointsToGrid::~ScanPointsToGrid()
{
}

void ScanPointsToGrid::scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr hokuyo_points (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *hokuyo_points);
    points::extract(hokuyo_points, width_meter_*0.5, height_meter_*0.5);
    pointToGrid(hokuyo_points, cost_);
    grid_.header = msg->header;
    grid_pub_.publish(grid_);
    resetData();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_grid");
    ScanPointsToGrid spg;
    ros::spin();
    return 0;
}
