#include "ans_grid_map_fusion/EvidentialFusion.hpp"

namespace ans_grid_map_fusion
{

    //   EvidentialFusion::EvidentialFusion(){}
      bool EvidentialFusion::configure(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger){
        this->logging_interface_ = node_logger;
        return true;
      }
      bool EvidentialFusion::update(const grid_map::GridMap msg, const std::vector<double> reliability) {
        // RCLCPP_INFO( this->logging_interface_->get_logger(), "reliability");
        return true;
      };

}  // namespace fusion_base


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ans_grid_map_fusion::EvidentialFusion, ans_grid_map_fusion::Fusion)
