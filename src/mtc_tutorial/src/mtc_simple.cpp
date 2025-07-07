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
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
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

  //plannners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

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


  // 3. close gripper
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("Close Gripper", interpolation_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("hande_closed");
  task.add(std::move(stage_close_hand));

  // 4. move to home
  auto stage_move_to_home = std::make_unique<mtc::stages::MoveTo>("Move to Home", sampling_planner);
  stage_move_to_home->setGroup(arm_group_name);
  stage_move_to_home->setGoal("moveit_home");
  task.add(std::move(stage_move_to_home));



// 5. move to pick
  // {
  //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
  //       "move to place",
  //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
  //                                                 { hand_group_name, sampling_planner } });
  //   stage_move_to_place->setTimeout(5.0);
  //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  //   task.add(std::move(stage_move_to_place));
  // }




  // // 5. move to pick
  
  // // Sample grasp pose
  // auto stage_grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  // stage_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
  // stage_grasp->properties().set("marker_ns", "grasp_pose");
  // stage_grasp->setPreGraspPose("hande_open");
  // stage_grasp->setObject("object");
  // stage_grasp->setAngleDelta(M_PI / 12);
  // stage_grasp->setMonitoredStage(current_state_ptr);  // Hook into current state
  
  // Eigen::Isometry3d grasp_frame_transform;
  // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
  //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
  //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  // grasp_frame_transform.linear() = q.matrix();
  // grasp_frame_transform.translation().z() = 0.1;


  //   // Compute IK
  // auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage_grasp));
  // wrapper->setMaxIKSolutions(8);
  // wrapper->setMinSolutionDistance(1.0);
  // wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  // wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  // wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  // task.add(std::move(wrapper));

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