#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/vision_stages.hpp"
#include "mtc_pipeline/action/vision_move_to_action.hpp"

using VisionMoveToAction = mtc_pipeline::action::VisionMoveToAction;
using BaseServer = BaseActionServer<VisionMoveToAction, VisionStages>;

class VisionActionServer : public BaseServer {
public:
  VisionActionServer() : BaseServer("vision_action_server", "vision_move_to_action") {}

protected:
  nlohmann::json goal_to_step(const VisionMoveToAction::Goal& goal) override {
    nlohmann::json step;
    step["tag_id"] = goal.tag_id;
    step["approach_distance"] = goal.approach_distance;
    step["timeout"] = goal.timeout;
    step["approach_direction"] = goal.approach_direction;
    step["use_preset_height"] = goal.use_preset_height;
    step["preset_height"] = goal.preset_height;
    step["planning_type"] = "joint";  // Default to joint planning
    return step;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}