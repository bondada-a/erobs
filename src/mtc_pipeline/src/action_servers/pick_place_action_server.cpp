#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"

class PickPlaceActionServer : public BaseActionServer<mtc_pipeline::action::PickPlaceAction, PickPlaceStages>
{
public:
    PickPlaceActionServer() : BaseActionServer("pick_place_action_server", "pick_place_action") {}
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
