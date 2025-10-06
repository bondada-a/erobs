#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/moveto_stages.hpp"
#include "mtc_pipeline/action/move_to_action.hpp"

class MoveToActionServer : public BaseActionServer<mtc_pipeline::action::MoveToAction, MoveToStages>
{
public:
    MoveToActionServer() : BaseActionServer("moveto_action_server", "moveto_action") {}

protected:
    nlohmann::json goal_to_step(const mtc_pipeline::action::MoveToAction::Goal& goal) override
    {
        nlohmann::json step;
        // Only add fields that are actually used
        if (!goal.target.empty()) {
            step["target"] = goal.target;
        }
        if (!goal.planning_type.empty()) {
            step["planning_type"] = goal.planning_type;
        }
        if (!goal.direction.empty()) {
            step["direction"] = goal.direction;
            step["distance"] = goal.distance;
        }
        return step;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
