#ifndef __SCAN_TO_GRID_H
#define __SCAN_TO_GRID_H

#include <sensor_msgs/PointCloud2.h>
#include "localmap_maker/gridmap_handler.h"

class ScanToGrid : public GridMapHandler
{
public:
    ScanToGrid();
    ~ScanToGrid();
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr&);
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher grid_pub_;
    const int8_t cost_;
};

#endif // __SCAN_TO_GRID_H
