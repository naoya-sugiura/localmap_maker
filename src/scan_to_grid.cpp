#include <pcl_ros/point_cloud.h>
#include "localmap_maker/scan_to_grid.h"
#include "localmap_maker/processing_pointcloud.h"

LaserScanHandler::LaserScanHandler()
    : cost_(100)
{
    cloud_sub_ = nh_.subscribe("/points/scan", 1, &LaserScanHandler::hokuyoCallback, this);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid/scan", 1);
}

LaserScanHandler::~LaserScanHandler()
{
}

void LaserScanHandler::hokuyoCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout << "scan to grid" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr hokuyo_points (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *hokuyo_points);
    points::extract(hokuyo_points, width_meter_*0.5, height_meter_*0.5);
    pointToGrid(hokuyo_points, cost_);
    grid_.header = msg->header;
    std::cout << "scan grid pub" << std::endl;
    grid_pub_.publish(grid_);
    dataReset();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_grid");
    LaserScanHandler lsh;
    ros::spin();
    return 0;
}
