#include <pcl_ros/point_cloud.h>

#include "localmap_maker/velodyne_points_to_grid.h"
#include "localmap_maker/processing_pointcloud.h"

VelodynePointsToGrid::VelodynePointsToGrid()
    : curvature_cost_(100), ground_cost_(50), obstacles_cost_(100), mask_(Flags::CURVATURE | Flags::GROUND | Flags::OBSTACLES)
{
    cloud_curvature_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/curvature", 1, boost::bind(&VelodynePointsToGrid::velodyneCallback, this, _1, Flags::CURVATURE));
    cloud_ground_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/ground", 1, boost::bind(&VelodynePointsToGrid::velodyneCallback, this, _1, Flags::GROUND));
    cloud_obstacles_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/obstacles", 1, boost::bind(&VelodynePointsToGrid::velodyneCallback, this, _1, Flags::OBSTACLES));

    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid/velodyne", 1);
}

VelodynePointsToGrid::~VelodynePointsToGrid()
{
}

void VelodynePointsToGrid::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const Flags::Type dataType)
{
    static Flags isUpdated = 0;
    if(dataType & isUpdated){
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *velodyne_pc);
    if(dataType & (Flags::GROUND | Flags::OBSTACLES)){
        points::downsample(velodyne_pc);
    }
    points::transform(velodyne_pc, msg->header.frame_id, msg->header.stamp);
    points::extract(velodyne_pc, width_meter_*0.5, height_meter_*0.5);
    pointToGrid(velodyne_pc, getCost(dataType));
    isUpdated = isUpdated | dataType;
    if((isUpdated & mask_) == mask_){
        grid_.header = msg->header;
        grid_pub_.publish(grid_);
        isUpdated = isUpdated & ~mask_;
        resetData();
    }
}

int8_t VelodynePointsToGrid::getCost(const Flags::Type dataType) const
{
    switch(dataType){
        case Flags::CURVATURE:
            return curvature_cost_;
        case Flags::GROUND:
            return ground_cost_;
        case Flags::OBSTACLES:
            return obstacles_cost_;
        default:
            return 0;
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_points_to_grid");
    VelodynePointsToGrid vpg;
    ros::spin();
    return 0;
}
