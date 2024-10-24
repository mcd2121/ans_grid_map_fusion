#ifndef ANS_GRID_MAP_FUSION_FUSION_HPP_
#define ANS_GRID_MAP_FUSION_FUSION_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <pluginlib/class_loader.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace ans_grid_map_fusion
{
  class Fusion
  {
    public:
      virtual void update(const grid_map_msgs::msg::GridMap gridmap, const std::vector<double> reliability) = 0;
      virtual ~Fusion(){}

    protected:
      Fusion(){}
  };
}  // namespace fusion_base

#endif  // ANS_GRID_MAP_FUSION_FUSION_HPP_