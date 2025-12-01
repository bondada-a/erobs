#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/vision_pick_place_stages.hpp"
#include "mtc_pipeline/action/vision_pick_place_action.hpp"

class VisionPickPlaceActionServer
  : public BaseActionServer<mtc_pipeline::action::VisionPickPlaceAction, VisionPickPlaceStages>
{
public:
  VisionPickPlaceActionServer()
    : BaseActionServer("vision_pick_place_action_server", "vision_pick_place_action") {}
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionPickPlaceActionServer>();
  node->initialize_stages();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
