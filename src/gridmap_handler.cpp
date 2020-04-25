#include "localmap_maker/gridmap_handler.h"

GridMapHandler::GridMapHandler()
    : local_nh_("~")
{
    local_nh_.param("resolution", grid_.info.resolution, {0.1});
    local_nh_.param("width", width_meter_, {20});
    local_nh_.param("height", height_meter_, {20});
    gridInitialization();
}

GridMapHandler::~GridMapHandler()
{
}

unsigned int GridMapHandler::getIndexFromPoint(const float x, const float y) const
{
    unsigned int i = getIFromX(x);
    unsigned int j = getJFromY(y);
    return getIndex(i, j);
}

void GridMapHandler::gridInitialization()
{
    grid_.info.width = static_cast<uint32_t>(width_meter_ / grid_.info.resolution + 0.5);
    grid_.info.height = static_cast<uint32_t>(height_meter_ / grid_.info.resolution + 0.5);
    grid_.info.origin.position.x = -width_meter_*0.5;
    grid_.info.origin.position.y = -height_meter_*0.5;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.x = 0.0;
    grid_.info.origin.orientation.y = 0.0;
    grid_.info.origin.orientation.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(grid_.info.width * grid_.info.height, 0);
}

void GridMapHandler::gridToGrid(const std::vector<int8_t>& grid)
{
    const auto grid_size = grid.size();
    for(size_t i=0; i<grid_size; ++i){
        if(grid.at(i) > grid_.data.at(i)){
            grid_.data.at(i) = grid.at(i);
        }
    }
}

void GridMapHandler::pointToGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, const int8_t cost)
{
    for(const auto& point : pc->points){
        grid_.data.at(getIndexFromPoint(point.x, point.y)) = cost;
    }
}
