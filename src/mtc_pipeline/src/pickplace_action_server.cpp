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
        step["pick_pose"] = goal.pick_pose;
        step["place_pose"] = goal.place_pose;
        step["planning_type"] = goal.planning_type;

        // Create pick and place poses arrays for the stage
        step["pick_poses"] = {goal.pick_pose + "_approach", goal.pick_pose};
        step["place_poses"] = {goal.place_pose + "_approach", goal.place_pose};

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
