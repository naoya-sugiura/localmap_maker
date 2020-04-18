#ifndef __PROCESS_POINTCLOUD_H
#define __PROCESS_POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <string>

namespace points
{

void downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr&);
void extract(pcl::PointCloud<pcl::PointXYZI>::Ptr&, float, float);
void transform(pcl::PointCloud<pcl::PointXYZI>::Ptr&, std::string, ros::Time);

} //namespace points

#endif // __PROCESS_POINTCLOUD_H
