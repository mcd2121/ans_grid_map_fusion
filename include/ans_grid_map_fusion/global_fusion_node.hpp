#ifndef ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
#define ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <string>

namespace ans_grid_map_fusion {

class GlobalFusion : public rclcpp::Node
{
public:
    GlobalFusion();
    virtual ~GlobalFusion();
private:

    void on_gridmap_cb(const std_msgs::msg::String::SharedPtr msg, const std::string &topic_name);
    void load_parameters();

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

    // Parameters
    struct LayerReliability
    {
        std::string name;
        double reliability;
    };

    struct GridMapConfig
    {
        std::string topic;
        std::vector<LayerReliability> layers;
    };

    std::map<std::string, GridMapConfig> grid_map_configs_;
    std::string output_topic_;

};





} // namespace
#endif  // ANS_GRID_MAP_FUSION_GLOBAL__FUSION_NODE_HPP_
