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
      Fusion(){}
      virtual ~Fusion(){}
      virtual bool configure(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger) = 0;
      virtual bool update(const grid_map::GridMap msg, const std::vector<double> reliability) = 0;

    protected:
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

  };
}  // namespace fusion

#endif  // ANS_GRID_MAP_FUSION_FUSION_HPP_