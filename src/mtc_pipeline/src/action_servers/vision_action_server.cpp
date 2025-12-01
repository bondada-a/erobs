#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/vision_stages.hpp"
#include "mtc_pipeline/action/vision_move_to_action.hpp"

class VisionActionServer : public BaseActionServer<mtc_pipeline::action::VisionMoveToAction, VisionStages>
{
public:
    VisionActionServer() : BaseActionServer("vision_action_server", "vision_move_to_action") {}
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
