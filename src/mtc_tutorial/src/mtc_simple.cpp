#include <rclcpp/rclcpp.hpp> 
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> 
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h> 
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/robot_state/conversions.h>  

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_logger");

namespace mtc = moveit::task_constructor;


class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::doTask()
{
  
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Stage initialization failed: " << e.what());
    return;
  }
  if (!task_.plan(5))                                 // how many plans to generate
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Pick and Place Task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_arm";
  const auto& hand_group_name = "hande_gripper";  //todo : make this a parameter for tool exchange
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  //plannners  //@todo : edit parameters for safety  
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.1);
  sampling_planner->setMaxAccelerationScalingFactor(0.1);
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.1);
  interpolation_planner->setMaxAccelerationScalingFactor(0.1);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(.005);

  // 1. get current state

  mtc::Stage* current_state_ptr = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // 2. open gripper
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("Open Gripper", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("hande_open");
  task.add(std::move(stage_open_hand));

  //3. move to pick
  std::map<std::string, double> joint_goal = {
  { "shoulder_pan_joint", -14.89 * M_PI / 180.0 },
  { "shoulder_lift_joint", -109.96 * M_PI / 180.0 },
  { "elbow_joint", -123.44 * M_PI / 180.0 },
  { "wrist_1_joint", -123.76 * M_PI / 180.0 },
  { "wrist_2_joint", 41.99 * M_PI / 180.0 },
  { "wrist_3_joint", -1.65 * M_PI / 180.0 }
  };

  auto stage_move_to_pick = std::make_unique<mtc::stages::MoveTo>("move to pick (joint map)", interpolation_planner);
  stage_move_to_pick->setGroup("ur_arm");
  stage_move_to_pick->setGoal(joint_goal);

  task.add(std::move(stage_move_to_pick));

  // 4. Move forward (+X in flange frame) by 0.5 meters
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach in +X (flange)", cartesian_planner);
  stage_approach->properties().set("marker_ns", "approach");
  stage_approach->properties().set("link", hand_frame);  
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

  // Set direction: +X in flange frame
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = hand_frame;
  direction.vector.x = 1.0;
  direction.vector.y = 0.0;
  direction.vector.z = 0.0;

  stage_approach->setDirection(direction);
  stage_approach->setMinMaxDistance(0.1,0.12);

  task.add(std::move(stage_approach));

    
  // 4.3 Allow Collision b/w gripper and object
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
      stage->allowCollisions("sample_holder",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      task.add(std::move(stage)); 
    }
    

    // 4.4 Close Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("hande_closed");
      task.add(std::move(stage));
    }
    
  
    // 4.5 Attach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("sample_holder", hand_frame);
      // attach_object_stage = stage.get();
      task.add(std::move(stage));
    }
    

    // 4.6 Pickup Retreat

      // 4. Move forward (+X in flange frame) by 0.5 meters
    auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat in -X (flange)", cartesian_planner);
    stage_retreat->properties().set("marker_ns", "retreat");
    stage_retreat->properties().set("link", hand_frame);
    stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

    // Set direction: -X in flange frame
    direction.header.frame_id = hand_frame;
    direction.vector.x = -1.0;
    direction.vector.y = 0.0;
    direction.vector.z = 0.0;

    stage_retreat->setDirection(direction);
    stage_retreat->setMinMaxDistance(0.1,0.12);  // exact 0.5 m forward

    task.add(std::move(stage_retreat));
  
  return task;
}




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}