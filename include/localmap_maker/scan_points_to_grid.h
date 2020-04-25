#ifndef __SCAN_POINTS_TO_GRID_H
#define __SCAN_POINTS_TO_GRID_H

#include <sensor_msgs/PointCloud2.h>
#include "localmap_maker/gridmap_handler.h"

class ScanPointsToGrid : public GridMapHandler
{
public:
    ScanPointsToGrid();
    ~ScanPointsToGrid();
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr&);
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher grid_pub_;
    const int8_t cost_;
};

#endif // __SCAN_POINTS_TO_GRID_H
