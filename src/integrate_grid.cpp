#include "localmap_maker/integrate_grid.h"

GridIntegrator::GridIntegrator()
    : private_nh_("~"), mask_(Flags::VELODYNE | Flags::HOKUYO)
{
    private_nh_.param("expand_range", expand_range_, {0.5});

    grid_velodyne_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/grid/velodyne", 1, boost::bind(&GridIntegrator::gridCallback, this, _1, Flags::VELODYNE));
    grid_hokuyo_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/grid/scan", 1, boost::bind(&GridIntegrator::gridCallback, this, _1, Flags::HOKUYO));
    localmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/localmap", 1);

}

GridIntegrator::~GridIntegrator()
{
}

void GridIntegrator::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, const Flags::Type flags)
{
    static Flags update_flag = 0;
    if(flags & update_flag){
        return;
    }
    gridToGrid(msg->data);
    update_flag = update_flag | flags;
    if((update_flag & mask_) == mask_){
        expand();
        grid_.header = msg->header;
        localmap_pub_.publish(grid_);
        update_flag = update_flag & ~mask_;
        resetData();
    }
}

void GridIntegrator::expand()
{
    const uint16_t cell_radius = static_cast<uint16_t>(expand_range_ / grid_.info.resolution);
    std::vector<int8_t> expand_grid(grid_.data.size(), 0);
    for(uint16_t i = 0; i < grid_.info.width; ++i){
        for(uint16_t j = 0; j < grid_.info.height; ++j){
            const uint16_t base_index = getIndex(i, j);
            const int8_t data = grid_.data.at(base_index);
            if(data <= 0){
                continue;
            }
            expand_grid.at(base_index) = data;
            for(int16_t k = -cell_radius; k<=cell_radius; ++k){
                for(int16_t l = -cell_radius; l<=cell_radius; ++l){
                    if(!mapValid(i+k, j+l)){
                        continue;
                    }
                    const uint16_t curr_index = getIndex(i+k, j+l);
                    if(expand_grid.at(curr_index)){
                        expand_grid.at(curr_index) = std::max(expand_grid.at(curr_index), data);
                        continue;
                    }
                    double distance_from_robot = sqrt(std::pow(getXFromI(i+k), 2.0) + std::pow(getYFromJ(j+l), 2.0));
                    if(distance_from_robot < expand_range_){
                        if(grid_.data.at(curr_index) < 100){
                            expand_grid.at(curr_index) = 0;
                        }
                        continue;
                    }
                    if(sqrt(std::pow(k, 2.0) + std::pow(l, 2.0)) > cell_radius){
                        continue;
                    }
                    expand_grid.at(curr_index) = std::max(grid_.data.at(curr_index), data);
                }
            }
        }
    }
    grid_.data.swap(expand_grid);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "integrate_grid");
    GridIntegrator gi;
    ros::spin();
    return 0;
}
