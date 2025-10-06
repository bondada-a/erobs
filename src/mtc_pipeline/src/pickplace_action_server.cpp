#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/pick_place_stages.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"

class PickPlaceActionServer : public BaseActionServer<mtc_pipeline::action::PickPlaceAction, PickPlaceStages>
{
public:
    PickPlaceActionServer() : BaseActionServer("pickplace_action_server", "pickplace_action") {}

protected:
    nlohmann::json goal_to_step(const mtc_pipeline::action::PickPlaceAction::Goal& goal) override
    {
        nlohmann::json step;
        step["gripper"] = goal.gripper;
        step["planning_type"] = goal.planning_type;

        // Use explicit approach and target poses (no magic suffix)
        step["pick_poses"] = {goal.pick_approach, goal.pick_target};
        step["place_poses"] = {goal.place_approach, goal.place_target};

        return step;
    }
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
