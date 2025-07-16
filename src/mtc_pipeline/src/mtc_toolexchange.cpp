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

#include <ur_msgs/srv/set_payload.hpp> //depending on your driver
#include <future>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_logger");

namespace mtc = moveit::task_constructor;


class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  
  std::string operation_; 

  int    dock_number_{3};      // 1-5, default centre
  double dock_offset_y_{0.0}; 

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
  void changePayload(double mass, const geometry_msgs::msg::Vector3& cog);
  
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

  // ─── lateral dock-shift helper ────────────────────────────────
  std::unique_ptr<mtc::stages::MoveRelative>
  makeDockShiftStage(double offset,
                     const std::string& name,
                     const Planners& planners) const;


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


void MTCTaskNode::changePayload(double mass, const geometry_msgs::msg::Vector3& cog)
{
  auto client = node_->create_client<ur_msgs::srv::SetPayload>(
    "/io_and_status_controller/set_payload");
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(LOGGER, "set_payload service not available");
    return;
  }
  auto req = std::make_shared<ur_msgs::srv::SetPayload::Request>();
  req->mass              = static_cast<float>(mass);
  req->center_of_gravity = cog;

  // send the request (returns a std::shared_future)
  auto result_fut = client->async_send_request(req);

  // wait up to 2s for the response; we do NOT spin here
  if (result_fut.wait_for(std::chrono::seconds(2))
        != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "set_payload service call timed out");
    return;
  }

  // future is ready—get() will not block
  auto response = result_fut.get();
  if (response->success)
    RCLCPP_INFO(LOGGER,
      "Payload updated: mass=%.3f, cog=(%.3f,%.3f,%.3f)",
      mass, cog.x, cog.y, cog.z);
  else
    RCLCPP_ERROR(LOGGER, "set_payload returned success=false");
}





MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // 1) (Optional) declare a ROS param for the file path
  std::string cfg_file;

  node_->get_parameter_or("poses_file", cfg_file, std::string("/home/user/poses.json"));
  node_->get_parameter_or("operation", operation_, std::string("load"));
  
  // ─── read dock_number (defaults to 3) ────────────────────────────────────
  
  node_->get_parameter_or("dock_number", dock_number_, 3);

  if (dock_number_ < 1 || dock_number_ > 5) {
    RCLCPP_FATAL(LOGGER, "dock_number must be 1-5, got %d", dock_number_);
    throw std::runtime_error("invalid dock_number");
  }
  constexpr double DOCK_SPACING = 0.1524;               // m
  dock_offset_y_ = DOCK_SPACING * static_cast<double>(3 - dock_number_);


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

std::unique_ptr<mtc::stages::MoveRelative>
MTCTaskNode::makeDockShiftStage(double offset,
                                const std::string& name,
                                const MTCTaskNode::Planners& planners) const

{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planners.cartesian);
  stage->properties().set("marker_ns", "dock_shift");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
  stage->setMinMaxDistance(std::abs(offset), std::abs(offset));

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.y = (offset >= 0.0) ? 1.0 : -1.0;   // direction
  stage->setDirection(vec);
  return stage;
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

// ─── lateral shift to selected dock ───────────────────────────────
  if (std::abs(dock_offset_y_) > 1e-4)
    task.add( makeDockShiftStage(dock_offset_y_, "shift to dock", planners) );


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


  // — payload change: read desired values from your JSON or hard-code —
  // double new_mass = config_["gripper_mass"].get<double>();
  // auto cog_arr = config_["gripper_cog"];
  // geometry_msgs::msg::Vector3 cog;
  // cog.x = cog_arr[0].get<double>();
  // cog.y = cog_arr[1].get<double>();
  // cog.z = cog_arr[2].get<double>();

  // // call the helper to update the UR driver
  // changePayload(new_mass, cog);

  // detach from holder
  {
    auto stage_dock_detach =
        std::make_unique<mtc::stages::MoveRelative>("detach_holder", planners.cartesian);
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
  if (std::abs(dock_offset_y_) > 1e-4)
    task.add( makeDockShiftStage(dock_offset_y_, "shift to dock", planners) );


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
  // back to default -   // todo  - this should be added as a stage , currently does before plan
  // double default_mass = config_["default_payload_mass"].get<double>();
  // auto default_cog_arr = config_["default_payload_cog"];
  // geometry_msgs::msg::Vector3 cog;
  // cog.x = default_cog_arr[0].get<double>();
  // cog.y = default_cog_arr[1].get<double>();
  // cog.z = default_cog_arr[2].get<double>();

  // // Pass the Vector3, not the JSON array:
  // changePayload(default_mass, cog);

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