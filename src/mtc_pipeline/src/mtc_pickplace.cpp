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

#include <fstream>
#include <nlohmann/json.hpp>

#include <moveit/task_constructor/solvers/planner_interface.h>  

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>


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

  nlohmann::json config_;
  void loadConfig(const std::string& path);

};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::loadConfig(const std::string& path)
{
  std::ifstream ifs(path);
  if (!ifs)
    throw std::runtime_error("Unable to open config file: " + path);
  ifs >> config_;
}


MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // 1) (Optional) declare a ROS param for the file path
  std::string cfg_file;

  node_->get_parameter_or("poses_file", cfg_file, std::string("/home/user/poses.json"));

  // 2) load the JSON
  try
  {
    loadConfig(cfg_file);
    RCLCPP_INFO_STREAM(LOGGER, "Loaded poses from: " << cfg_file);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to load JSON config: " << e.what());
    throw;  // or handle the error however you like
  }

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
  sampling_planner->setMaxVelocityScalingFactor(0.2);
  sampling_planner->setMaxAccelerationScalingFactor(0.2);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.2);
  interpolation_planner->setMaxAccelerationScalingFactor(0.2);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(.001);
  cartesian_planner->setMinFraction(0.95);  // require only 95% of the path
  


  moveit_msgs::msg::Constraints wrist3_constraint;
  wrist3_constraint.name = "wrist3_upright";
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name      = "wrist_3_joint";
    jc.position        = 0.0;    // upright angle in radians
    jc.tolerance_above = 0.01;   // ±0.02 rad slack
    jc.tolerance_below = 0.01;
    jc.weight          = 1.0;    // enforce as a hard constraint
    wrist3_constraint.joint_constraints.push_back(jc);
  }




  // JSON
  auto addNamedMoveStage =
    [&](const std::string& label,
        const std::string& pose_key,
        const mtc::solvers::PlannerInterfacePtr& planner,
        const moveit_msgs::msg::Constraints* path_constraints = nullptr)
  {
    // 1) read the 6-element array from your JSON
    auto& angles_deg = config_["poses"][pose_key];
    if (!angles_deg.is_array() || angles_deg.size() != 6)
      throw std::runtime_error(pose_key + " must be an array of 6 numbers");

    // 2) convert to radians
    const std::vector<std::string> joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",      "wrist_3_joint"
    };
    std::map<std::string, double> joint_goal;
    for (size_t i = 0; i < 6; ++i)
      joint_goal[joint_names[i]] = angles_deg[i].get<double>() * M_PI / 180.0;

    // 3) make the MoveTo stage
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(arm_group_name);
    stage->setGoal(joint_goal);

    // 4) apply constraint only if provided
    if (path_constraints)
      stage->setPathConstraints(*path_constraints);

    task.add(std::move(stage));
  };


  
  


    
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

// 3. move to pickup_approach from JSON
  // 3a. approach the pickup pose
  addNamedMoveStage("move to pickup approach", "pickup_approach",sampling_planner);

  // 3b. move in to the actual pickup pose
  addNamedMoveStage("move to pickup", "pickup", cartesian_planner);

  
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

  addNamedMoveStage("pickup retreat", "pickup_approach", cartesian_planner );
  
    
  addNamedMoveStage("move to place", "place_approach",sampling_planner, &wrist3_constraint);


  addNamedMoveStage("place", "place", sampling_planner);

  {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("hande_open");
      task.add(std::move(stage));
  }

  addNamedMoveStage("place", "place_approach", cartesian_planner);




  
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