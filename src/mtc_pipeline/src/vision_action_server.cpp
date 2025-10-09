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
    step["timeout"] = goal.timeout;
    return step;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionActionServer>();
  node->initialize_stages();  // Initialize stages after node is created
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}