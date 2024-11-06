#include "ans_grid_map_fusion/global_fusion_node.hpp"

namespace ans_grid_map_fusion {

GlobalFusion::GlobalFusion() : Node("global_fusion_node")
{

    // Declare parameters
    this->declare_parameter<std::vector<std::string>>("input_gridmaps", {});
    this->declare_parameter<std::string>("output_topic", "/fused_map_global");
    this->declare_parameter<std::string>("fusion_policy", "ans_grid_map_fusion::EvidentialFusion");

    std::vector<std::string> input_gridmaps = this->get_parameter("input_gridmaps").as_string_array();
    output_topic_ = this->get_parameter("output_topic").as_string();
    fusion_policy_ = this->get_parameter("fusion_policy").as_string();

    for (const auto &gridmap_name : input_gridmaps)
    {
        this->declare_parameter<std::string>(gridmap_name + ".topic", "");
        this->declare_parameter<std::vector<std::string>>(gridmap_name + ".layers", {});

        std::string topic = this->get_parameter(gridmap_name + ".topic").as_string();
        std::vector<std::string> layers = this->get_parameter(gridmap_name + ".layers").as_string_array();

        GridMapConfig grid_map_config;
        grid_map_config.topic = topic;

        for (const auto &layer : layers)
        {
            this->declare_parameter<double>(gridmap_name + "." + layer + ".reliability", 1.0);
            double reliability = this->get_parameter(gridmap_name + "." + layer + ".reliability").as_double();
            grid_map_config.layer_reliablity[layer] = reliability;
        }
        grid_map_configs_[gridmap_name] = grid_map_config;
    }

    auto fusion_handle_ = plugin_loader_.createSharedInstance(fusion_policy_);
    fusion_handle_->configure(this->get_node_logging_interface());

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&GlobalFusion::parameters_cb, this, std::placeholders::_1));
    

    for (auto &&gridmap : grid_map_configs_)
    {
        std::function<void(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)> fnc = std::bind(
            &GlobalFusion::grid_map_cb, this, std::placeholders::_1, gridmap.second.topic);
        
        subscriptions_[gridmap.second.topic] = this->create_subscription<grid_map_msgs::msg::GridMap>(
            gridmap.second.topic, rclcpp::QoS{1}.best_effort(), fnc);
    }
    

}

GlobalFusion::~GlobalFusion() {}



rcl_interfaces::msg::SetParametersResult GlobalFusion::parameters_cb(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    std::vector<std::string> keys;
    std::string s;
    for (const auto &param: parameters)
    {
        std::string name = param.get_name();
        std::stringstream ss(name);
        
        if (name.find(".") == std::string::npos) {
            continue;
        }

        while (getline(ss, s, '.'))
        {
            keys.push_back(s);
        }

        grid_map_configs_[keys[0]].layer_reliablity[keys[1]] = param.as_double();

    break;
    }
    return result;
}

void GlobalFusion::grid_map_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg, const std::string &grid_map_name)
{
    RCLCPP_INFO_STREAM(this->get_logger(), grid_map_name);

    grid_map::GridMap inputMap;
    grid_map::GridMapRosConverter::fromMessage(*msg, inputMap);

    std::vector<double> reliabilities = {5.1,6.3,6.6};

    fusion_handle_->update(inputMap, reliabilities);
    
}



} // namespace



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ans_grid_map_fusion::GlobalFusion>());
  rclcpp::shutdown();
  return 0;
}

