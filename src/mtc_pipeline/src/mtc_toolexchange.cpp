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
  
  std::string operation_; 

  void doTask();

  

private:
  mtc::Task createLoadTask();
  mtc::Task createDockTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  nlohmann::json config_;
  void loadConfig(const std::string& path);

  void addNamedMoveStage(
        mtc::Task& task,
        const std::string& label,
        const std::string& pose_key,
        const mtc::solvers::PlannerInterfacePtr& planner,
        const moveit_msgs::msg::Constraints* path_constraints = nullptr
    );
  
  // constants or parameters
  std::string arm_group_name   = "ur_arm";
  std::string hand_group_name  = "hande_gripper";
  std::string hand_frame       = "flange";

  // helper to set up task & planners
  struct Planners {
    mtc::solvers::PipelinePlannerPtr        sampling;
    mtc::solvers::JointInterpolationPlannerPtr interpolation;
    mtc::solvers::CartesianPathPtr          cartesian;
  };
  Planners initCommon(mtc::Task& task);

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


MTCTaskNode::Planners MTCTaskNode::initCommon(mtc::Task& task)
{
  // — set the three properties once —
  task.setProperty("group",     arm_group_name);
  task.setProperty("eef",       hand_group_name);
  task.setProperty("ik_frame",  hand_frame);

  // — build your planners once —
  Planners p;
  p.sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  p.sampling->setMaxVelocityScalingFactor(0.2);
  p.sampling->setMaxAccelerationScalingFactor(0.2);

  p.interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  p.interpolation->setMaxVelocityScalingFactor(0.2);
  p.interpolation->setMaxAccelerationScalingFactor(0.2);

  p.cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  p.cartesian->setMaxVelocityScalingFactor(0.2);
  p.cartesian->setMaxAccelerationScalingFactor(0.2);
  p.cartesian->setStepSize(0.001);
  p.cartesian->setMinFraction(0.95);

  return p;
}



MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // 1) (Optional) declare a ROS param for the file path
  std::string cfg_file;

  node_->get_parameter_or("poses_file", cfg_file, std::string("/home/user/poses.json"));
  node_->get_parameter_or("operation", operation_, std::string("load"));
  
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


void MTCTaskNode::addNamedMoveStage(
    mtc::Task& task,
    const std::string& label,
    const std::string& pose_key,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const moveit_msgs::msg::Constraints* path_constraints)
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
    stage->setGroup("ur_arm");  // or use a member variable if you parametrize it
    stage->setGoal(joint_goal);

    // 4) apply constraint only if provided
    if (path_constraints)
        stage->setPathConstraints(*path_constraints);

    task.add(std::move(stage));
}




void MTCTaskNode::doTask()
{
  if (operation_ == "load") {
    task_ = createLoadTask();
  } else if (operation_ == "dock") {
    task_ = createDockTask();
  } else {
    RCLCPP_ERROR_STREAM(LOGGER, "Unknown operation: " << operation_);
    return;
  }

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Stage initialization failed: " << e.what());
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
}




mtc::Task MTCTaskNode::createLoadTask()
{
  mtc::Task task;
  task.stages()->setName("Load Task");
  task.loadRobotModel(node_);

  auto planners = initCommon(task);

  // 1. get current state

  mtc::Stage* current_state_ptr = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

// 3. move to load_approach from JSON
  addNamedMoveStage(task,"move to load approach", "load_approach",planners.sampling);

// attach to dock
  {
    auto stage_dock_connect =
        std::make_unique<mtc::stages::MoveRelative>("attach_tool", planners.cartesian);
    stage_dock_connect->properties().set("marker_ns", "approach_object");
    stage_dock_connect->properties().set("link", hand_frame);
    stage_dock_connect->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_dock_connect->setMinMaxDistance(0.1, 0.1);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.x= 1.0;
    stage_dock_connect->setDirection(vec);
    task.add(std::move(stage_dock_connect));
  }

  // detach from dock
  {
    auto stage_dock_detach =
        std::make_unique<mtc::stages::MoveRelative>("detach_dock", planners.cartesian);
    stage_dock_detach->properties().set("marker_ns", "approach_object");
    stage_dock_detach->properties().set("link", hand_frame);
    stage_dock_detach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_dock_detach->setMinMaxDistance(0.15, 0.15);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.z= -1.0;
    stage_dock_detach->setDirection(vec);
    task.add(std::move(stage_dock_detach));
  }

  {
    auto stage_move_up =
        std::make_unique<mtc::stages::MoveRelative>("move_up", planners.cartesian);
    stage_move_up->properties().set("marker_ns", "approach_object");
    stage_move_up->properties().set("link", hand_frame);
    stage_move_up->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_move_up->setMinMaxDistance(0.2, 0.2);

    // Set hand up direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.x= -1.0;
    stage_move_up->setDirection(vec);
    task.add(std::move(stage_move_up));
  }
  
  
  return task;
}



mtc::Task MTCTaskNode::createDockTask()
{
  mtc::Task task;
  task.stages()->setName("Dock Task");
  task.loadRobotModel(node_);

  auto planners = initCommon(task);
  
  // 1. get current state

  mtc::Stage* current_state_ptr = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));


  addNamedMoveStage(task,"move to dock approach", "dock_approach",planners.sampling);

// align with holder
  {
    auto stage_dock_connect =
        std::make_unique<mtc::stages::MoveRelative>("align_holder", planners.cartesian);
    stage_dock_connect->properties().set("marker_ns", "approach_object");
    stage_dock_connect->properties().set("link", hand_frame);
    stage_dock_connect->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_dock_connect->setMinMaxDistance(0.2, 0.2);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.x= 1.0;
    stage_dock_connect->setDirection(vec);
    task.add(std::move(stage_dock_connect));
  }

  // move to holder
  {
    auto stage_dock_detach =
        std::make_unique<mtc::stages::MoveRelative>("detach_tool", planners.cartesian);
    stage_dock_detach->properties().set("marker_ns", "approach_object");
    stage_dock_detach->properties().set("link", hand_frame);
    stage_dock_detach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_dock_detach->setMinMaxDistance(0.15, 0.15);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.z= 1.0;
    stage_dock_detach->setDirection(vec);
    task.add(std::move(stage_dock_detach));
  }

  {
    auto stage_move_up =
        std::make_unique<mtc::stages::MoveRelative>("dock connect", planners.cartesian);
    stage_move_up->properties().set("marker_ns", "approach_object");
    stage_move_up->properties().set("link", hand_frame);
    stage_move_up->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_move_up->setMinMaxDistance(0.1, 0.1);

    // Set hand up direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame; 
    vec.vector.x= -1.0;
    stage_move_up->setDirection(vec);
    task.add(std::move(stage_move_up));
  }
  
  
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