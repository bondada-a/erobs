#include "mtc_pipeline/base_action_server.hpp"
#include "mtc_pipeline/pipettor_stages.hpp"
#include "mtc_pipeline/action/pipettor_action.hpp"

class PipettorActionServer : public BaseActionServer<mtc_pipeline::action::PipettorAction, PipettorStages>
{
public:
    PipettorActionServer() : BaseActionServer("pipettor_action_server", "pipettor_action") {}

protected:
    nlohmann::json goal_to_step(const mtc_pipeline::action::PipettorAction::Goal& goal) override
    {
        nlohmann::json step;
        step["operation"] = goal.operation;
        step["volume_pct"] = goal.volume_pct;

        // Convert LED color to JSON
        nlohmann::json led_color;
        led_color["r"] = goal.led_color.r;
        led_color["g"] = goal.led_color.g;
        led_color["b"] = goal.led_color.b;
        led_color["a"] = goal.led_color.a;
        step["led_color"] = led_color;

        return step;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PipettorActionServer>();
    node->initialize_stages();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
