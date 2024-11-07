#ifndef ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
#define ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <pluginlib/class_loader.hpp>
#include "ans_grid_map_fusion/fusion.hpp"
#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace ans_grid_map_fusion {

class GlobalFusion : public rclcpp::Node
{
public:
    GlobalFusion();
    virtual ~GlobalFusion();

    bool update(const grid_map::GridMap &mapIn, const std::vector<double> reliability);

private:
    bool global_fusion(const grid_map::GridMap &new_map);
    grid_map::Length calculateNewGlobalSize(const grid_map::GridMap &new_map, const grid_map::GridMap &global_map);
    grid_map::Position calculateNewGlobalPosition(const grid_map::GridMap &new_map, const grid_map::GridMap &global_map);
    void grid_map_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg, const std::string &grid_map_name);
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    std::map<std::string, rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr> subscriptions_;
    pluginlib::ClassLoader<ans_grid_map_fusion::Fusion> plugin_loader_{"ans_grid_map_fusion", "ans_grid_map_fusion::Fusion"};
    std::shared_ptr<ans_grid_map_fusion::Fusion> fusion_handle_;
    // Parameters
    rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &parameters);
    
    grid_map::GridMap global_map_;
    grid_map::GridMap map_buffer_;

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
    std::string fusion_policy_;

};





} // namespace
#endif  // ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
