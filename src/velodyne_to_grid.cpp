#include <pcl_ros/point_cloud.h>

#include "localmap_maker/velodyne_to_grid.h"
#include "localmap_maker/processing_pointcloud.h"

GroundToGrid::GroundToGrid()
    : ground_cost_(50), curvature_cost_(100), obstacles_cost_(100), mask_(Flags::CURVATURE | Flags::GROUND | Flags::OBSTACLES)
{
    cloud_curvature_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/curvature", 1, boost::bind(&GroundToGrid::velodyneCallback, this, _1, Flags::CURVATURE));
    cloud_ground_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/ground", 1, boost::bind(&GroundToGrid::velodyneCallback, this, _1, Flags::GROUND));
    cloud_obstacles_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/obstacles", 1, boost::bind(&GroundToGrid::velodyneCallback, this, _1, Flags::OBSTACLES));

    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid/velodyne", 1);
}

GroundToGrid::~GroundToGrid()
{
}

void GroundToGrid::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const Flags::Type flags)
{
    static Flags update_flag = 0;
    if(flags & update_flag){
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *velodyne_pc);
    if(flags & (Flags::GROUND | Flags::OBSTACLES)){
        points::downsample(velodyne_pc);
    }
    points::transform(velodyne_pc, msg->header.frame_id, msg->header.stamp);
    points::extract(velodyne_pc, width_meter_*0.5, height_meter_*0.5);
    pointToGrid(velodyne_pc, getCost(flags));
    update_flag = update_flag | flags;
    if((update_flag & mask_) == mask_){
        grid_.header = msg->header;
        grid_pub_.publish(grid_);
        update_flag = update_flag & ~mask_;
        dataReset();
    }
}

int8_t GroundToGrid::getCost(const Flags::Type flags) const
{
    switch(flags){
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
    ros::init(argc, argv, "velodyne_to_grid");
    GroundToGrid gt;
    ros::spin();
    return 0;
}
