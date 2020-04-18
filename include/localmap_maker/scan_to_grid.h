#ifndef __LASERSCAN_HANDLER_H
#define __LASERSCAN_HANDLER_H

#include <sensor_msgs/PointCloud2.h>
#include "localmap_maker/gridmap_handler.h"

class LaserScanHandler : public GridMapHandler
{
public:
    LaserScanHandler();
    ~LaserScanHandler();
    void hokuyoCallback(const sensor_msgs::PointCloud2::ConstPtr&);
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher grid_pub_;
    const int8_t cost_;
};

#endif //LaserScanHandler
