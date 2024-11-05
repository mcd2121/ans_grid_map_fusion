#include "ans_grid_map_fusion/EvidentialFusion.hpp"

namespace ans_grid_map_fusion
{

    //   EvidentialFusion::EvidentialFusion(){}

      void EvidentialFusion::update(const grid_map_msgs::msg::GridMap gridmap, const std::vector<double> reliability) {
            
      };

}  // namespace fusion_base


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ans_grid_map_fusion::EvidentialFusion, ans_grid_map_fusion_base::Fusion)
