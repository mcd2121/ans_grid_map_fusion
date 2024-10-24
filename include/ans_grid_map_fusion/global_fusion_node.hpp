#ifndef ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
#define ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace ans_grid_map_fusion {

class GlobalFusion : public rclcpp::Node
{
public:
    GlobalFusion();
    virtual ~GlobalFusion();
private:

    void grid_map_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg, const std::string &grid_map_name);

    void load_parameters();

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::map<std::string, rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr> subscriptions_;


    // Parameters
    rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &parameters);

    struct LayerReliability
    {
        std::string name;
        double reliability;
    };

    struct GridMapConfig
    {
        std::string topic;
        std::map<std::string, double> layer_reliablity;
    };

    std::map<std::string, GridMapConfig> grid_map_configs_;
    std::string output_topic_;

};





} // namespace
#endif  // ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
