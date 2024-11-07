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
        grid_map_configs_[topic] = grid_map_config;
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
    
    for (const auto &layer : inputMap.getLayers()){
        if(grid_map_configs_[grid_map_name].layer_reliablity.find(layer) != grid_map_configs_[grid_map_name].layer_reliablity.end()){
            map_buffer_.add(layer,  inputMap[layer]);
        }
    }
    for (const auto &layer : map_buffer_.getLayers()){
        RCLCPP_INFO_STREAM(this->get_logger(), layer);
    }


    std::vector<double> reliabilities = {5.1,6.3,6.6};

    update(inputMap, reliabilities);
    
}

bool GlobalFusion::global_fusion(const grid_map::GridMap &new_map)
{
    double new_map_resolution = new_map.getResolution();
    grid_map::Position new_map_position = new_map.getPosition();  // Center of the new map

    double global_map_resolution = global_map_.getResolution();
    grid_map::Position global_map_position = global_map_.getPosition();

    grid_map::Length new_global_size = calculateNewGlobalSize(new_map, global_map_);
    grid_map::Position new_global_position = calculateNewGlobalPosition(new_map, global_map_);

    global_map_.setGeometry(new_global_size, global_map_resolution, new_global_position);

    // global_map_.addDataFrom(new_map, true, true, true, new_map.getLayers() );

    for (const auto &layer : new_map.getLayers()) {
        if (global_map_.exists(layer)) {
            for (grid_map::GridMapIterator it(new_map); !it.isPastEnd(); ++it) {
                const grid_map::Index index(*it);
                if (!std::isnan(new_map.at(layer, index))) {
                    // Only update the cells with valid data in the new map
                    global_map_.at(layer, index) = new_map.at(layer, index);
                }
            }

        } else {
            global_map_.add(layer, new_map[layer]);   // Add a new layer to global map
        }
    }
    return true;
}

grid_map::Length GlobalFusion::calculateNewGlobalSize(const grid_map::GridMap &new_map, const grid_map::GridMap &global_map)
{
    // Calculate the new size of the global map based on the incoming map
    grid_map::Position new_map_position = new_map.getPosition();
    grid_map::Length new_map_size = new_map.getLength();
    
    grid_map::Position global_map_position = global_map.getPosition();
    grid_map::Length global_map_size = global_map.getLength();

    // Calculate min/max bounds for both maps
    grid_map::Position new_map_min = new_map_position.array() - new_map_size / 2.0;
    grid_map::Position new_map_max = new_map_position.array() + new_map_size / 2.0;
    
    grid_map::Position global_map_min = global_map_position.array() - global_map_size / 2.0;
    grid_map::Position global_map_max = global_map_position.array() + global_map_size / 2.0;

    // Get new global map boundaries
    double min_x = std::min(new_map_min.x(), global_map_min.x());
    double min_y = std::min(new_map_min.y(), global_map_min.y());
    double max_x = std::max(new_map_max.x(), global_map_max.x());
    double max_y = std::max(new_map_max.y(), global_map_max.y());

    // Return new size
    return grid_map::Length(max_x - min_x, max_y - min_y);
}

grid_map::Position GlobalFusion::calculateNewGlobalPosition(const grid_map::GridMap &new_map, const grid_map::GridMap &global_map)
{
    // Calculate the new center of the global map based on the incoming map
    grid_map::Position new_map_position = new_map.getPosition();
    grid_map::Length new_map_size = new_map.getLength();
    
    grid_map::Position global_map_position = global_map.getPosition();
    grid_map::Length global_map_size = global_map.getLength();

    // Calculate min/max bounds for both maps
    grid_map::Position new_map_min = new_map_position.array() - new_map_size / 2.0;
    grid_map::Position new_map_max = new_map_position.array() + new_map_size / 2.0;
    
    grid_map::Position global_map_min = global_map_position.array() - global_map_size / 2.0;
    grid_map::Position global_map_max = global_map_position.array() + global_map_size / 2.0;

    // Get new global map boundaries
    double min_x = std::min(new_map_min.x(), global_map_min.x());
    double min_y = std::min(new_map_min.y(), global_map_min.y());
    double max_x = std::max(new_map_max.x(), global_map_max.x());
    double max_y = std::max(new_map_max.y(), global_map_max.y());

    // Calculate the new center position
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;

    return grid_map::Position(center_x, center_y);
}


bool GlobalFusion::update(const grid_map::GridMap &mapIn, const std::vector<double> reliability){
    RCLCPP_INFO_STREAM(this->get_logger(), reliability[0]);
    
}



} // namespace



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ans_grid_map_fusion::GlobalFusion>());
  rclcpp::shutdown();
  return 0;
}

