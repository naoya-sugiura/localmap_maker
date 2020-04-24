#ifndef __VELODYNE_POINTS_TO_GRID_H
#define __VELODYNE_POINTS_TO_GRID_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "localmap_maker/gridmap_handler.h"

class VelodynePointsToGrid : public GridMapHandler
{
public:
    class Flags {
    public:
        enum Type : unsigned int {
            CURVATURE = 1 << 0,
            GROUND = 1 << 1,
            OBSTACLES = 1 << 2
        };
        Flags(unsigned int val) : val_(static_cast<Type>(val)){}
        operator Type(){return val_;}
    private:
        Type val_;
    };
    VelodynePointsToGrid();
    ~VelodynePointsToGrid();
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr&, const Flags::Type);
    int8_t getCost(const Flags::Type) const;
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_curvature_sub_;
    ros::Subscriber cloud_ground_sub_;
    ros::Subscriber cloud_obstacles_sub_;
    ros::Publisher grid_pub_;
    const int8_t curvature_cost_;
    const int8_t ground_cost_;
    const int8_t obstacles_cost_;
    Flags mask_;
};

#endif // __VELODYNE_POINTS_TO_GRID_H
