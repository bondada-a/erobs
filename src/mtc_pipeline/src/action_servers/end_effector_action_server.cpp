#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/end_effector_stages.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"

class EndEffectorActionServer : public BaseActionServer<mtc_pipeline::action::EndEffectorAction, EndEffectorStages>
{
public:
    EndEffectorActionServer() : BaseActionServer("end_effector_action_server", "end_effector_action") {}
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
