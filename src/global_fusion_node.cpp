#include "ans_grid_map_fusion/global_fusion_node.hpp"

namespace ans_grid_map_fusion {

GlobalFusion::GlobalFusion() : Node("global_fusion_node")
{

    load_parameters();

}

GlobalFusion::~GlobalFusion() {
    // Destructor implementation
}

void GlobalFusion::load_parameters()
{
    // Declare parameters
    this->declare_parameter<std::vector<std::string>>("input_gridmaps", {});
    this->declare_parameter<std::string>("output_topic", "/fused_map_global");

    std::vector<std::string> input_gridmaps = this->get_parameter("input_gridmaps").as_string_array();
    output_topic_ = this->get_parameter("output_topic").as_string();

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
            grid_map_config.layers.push_back({layer, reliability});
        }

        grid_map_configs_[gridmap_name] = grid_map_config;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded parameters for grid maps.");
}


} // namespace



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ans_grid_map_fusion::GlobalFusion>());
  rclcpp::shutdown();
  return 0;
}
