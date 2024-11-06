#include "ans_grid_map_fusion/fusion.hpp"

namespace ans_grid_map_fusion
{
  class EvidentialFusion : public ans_grid_map_fusion::Fusion
  {
    public:
      EvidentialFusion(){};
      virtual bool configure(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger);

      virtual bool update(const grid_map::GridMap msg, const std::vector<double> reliability);
      virtual ~EvidentialFusion(){};
      
  };
}  // namespace fusion_base


