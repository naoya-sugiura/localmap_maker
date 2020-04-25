#ifndef __INTEGRATE_GRID_H
#define __INTEGRATE_GRID_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "localmap_maker/gridmap_handler.h"

class GridIntegrator : public GridMapHandler
{
public:
    class Flags {
    public:
        enum Type : unsigned int {
            VELODYNE = 1 << 0,
            HOKUYO = 1 << 1
        };
        Flags(unsigned int val) : val_(static_cast<Type>(val)){}
        operator Type(){return val_;}
    private:
        Type val_;
    };
    GridIntegrator();
    ~GridIntegrator();
    void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr&, const Flags::Type);
    void expand();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher localmap_pub_;
    ros::Subscriber grid_velodyne_sub_;
    ros::Subscriber grid_hokuyo_sub_;
    Flags mask_;
    double expand_range_;
};

#endif //__INTEGRATE_GRID_H
