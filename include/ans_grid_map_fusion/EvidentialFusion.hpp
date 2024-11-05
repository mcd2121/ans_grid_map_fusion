#include "ans_grid_map_fusion/fusion.hpp"

namespace ans_grid_map_fusion
{
  class EvidentialFusion : public ans_grid_map_fusion_base::Fusion
  {
    public:
      EvidentialFusion(){};
      virtual void update(const grid_map_msgs::msg::GridMap gridmap, const std::vector<double> reliability);
      virtual ~EvidentialFusion(){};

      
  };
}  // namespace fusion_base


