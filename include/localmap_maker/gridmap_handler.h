#ifndef __GRIDMAP_HANDLER_H
#define __GRIDMAP_HANDLER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GridMapHandler{
public:
    GridMapHandler();
    virtual ~GridMapHandler();
    void resetData(){std::fill(grid_.data.begin(), grid_.data.end(), 0.0);};
    unsigned int getIndexFromPoint(const float, const float) const;
    template <typename T> T getIndex(T i, T j) const;
    template<typename T> float getXFromI(T i) const;
    template<typename T> float getYFromJ(T j) const;
    template<typename T> unsigned int getIFromX(T x) const;
    template<typename T> unsigned int getJFromY(T y) const;
    template<typename T> bool mapValid(T i, T j) const;
    void gridInitialization();
    void gridToGrid(const std::vector<int8_t>&);
    void pointToGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, const int8_t);
protected:
    nav_msgs::OccupancyGrid grid_;
    ros::NodeHandle local_nh_;
    int width_meter_;
    int height_meter_;
};

template<typename T>
inline T GridMapHandler::getIndex(T i, T j) const
{
    return i + j*grid_.info.width;
}

template<typename T>
inline float GridMapHandler::getXFromI(T i) const
{
    return (i - grid_.info.width*0.5)*grid_.info.resolution;
}

template<typename T>
inline float GridMapHandler::getYFromJ(T j) const
{
    return (j - grid_.info.height*0.5)*grid_.info.resolution;
}

template<typename T>
inline unsigned int GridMapHandler::getIFromX(T x) const
{
    return floor( (x - grid_.info.origin.position.x)/grid_.info.resolution);
}

template<typename T>
inline unsigned int GridMapHandler::getJFromY(T y) const
{
    return floor( (y - grid_.info.origin.position.y)/grid_.info.resolution);
}

template<typename T>
inline bool GridMapHandler::mapValid(T i, T j) const
{
    return ((i >= 0) && (i < static_cast<T>(grid_.info.width)) && (j >= 0) && (j < static_cast<T>(grid_.info.height)));
}

#endif //__GRIDMAP_HANDLER_H
