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
#include <tf2_ros/static_transform_broadcaster.h>  // add this
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_logger");

namespace mtc = moveit::task_constructor;


class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

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

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "sample_holder";
  object.header.frame_id = "map";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.11, 0.028, 0.015 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0.44;
  pose.position.z = 0.14;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.60);        // roll=0, pitch=0, yaw=0.60 rad
  pose.orientation = tf2::toMsg(q);

  object.pose = pose;
  object.subframe_names.resize(5);
  object.subframe_poses.resize(5);
  object.subframe_names[0]= "holder";
  object.subframe_poses[0].position.x= -0.05;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  // 2) now broadcast a TF called “sample_holder” at that same pose
  // geometry_msgs::msg::TransformStamped tf_msg;
  // tf_msg.header.stamp = node_->now();
  // tf_msg.header.frame_id    = object.header.frame_id;   // “map”
  // tf_msg.child_frame_id     = object.id;                // “sample_holder”
  // tf_msg.transform.translation.x = pose.position.x;
  // tf_msg.transform.translation.y = pose.position.y;
  // tf_msg.transform.translation.z = pose.position.z;
  // tf_msg.transform.rotation      = pose.orientation;
  // tf_broadcaster_->sendTransform(tf_msg);
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

  if (!task_.plan(1))                                 // how many plans to generate
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  // RCLCPP_INFO_STREAM(LOGGER, "Generated " << task_.solutions().size() << " solution(s)");

  // std::size_t i = 0;
  // for (const auto& solution : task_.solutions())
  // {
  //   RCLCPP_INFO_STREAM(LOGGER, "Solution #" << i++ << ": cost = " << solution->cost());
  // }

  // // Do not execute anything
  // return;
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
  const auto& hand_frame = "robotiq_hande_end";

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

  // 3. move to pick
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });

  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator

 // 4. Pick Object - Serial Container
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    
    // 4.1 Pickup Approach
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("pickup approach", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.01);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame; 
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4.2 Generate Grasp Pose
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("hande_open");
      stage->setObject("sample_holder/holder");
      stage->setAngleDelta(2*M_PI);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = -0.02; // distance from flange to object

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(16);
      wrapper->setMinSolutionDistance(0.01);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }


      // 4.3 Allow Collision b/w gripper and object
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions("sample_holder",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    // 4.4 Close Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("hande_closed");
      grasp->insert(std::move(stage));
    }

    // 4.5 Attach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("sample_holder", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // 4.6 Pickup Retreat
    { 
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("pickup retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.1);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set backward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  // // 5. Move to Place
  // {
  //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
  //       "move to place",
  //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
  //                                                 { hand_group_name, sampling_planner } });
  //   stage_move_to_place->setTimeout(5.0);
  //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  //   task.add(std::move(stage_move_to_place));
  // }

  // // 6. Place Object - Serial Container
  // {
  //   auto place = std::make_unique<mtc::SerialContainer>("place object");
  //   task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  //   place->properties().configureInitFrom(mtc::Stage::PARENT,
  //                                         { "eef", "group", "ik_frame" });
   
  //   // 6.1 Place Approach
  //   {
  //     // Sample place pose
  //     auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT);
  //     stage->properties().set("marker_ns", "place_pose");
  //     stage->setObject("sample_holder");

  //     geometry_msgs::msg::PoseStamped target_pose_msg;
  //     target_pose_msg.header.frame_id = "sample_holder";
  //     target_pose_msg.pose.position.y = 0.5;
  //     target_pose_msg.pose.orientation.w = 1.0;
  //     stage->setPose(target_pose_msg);
  //     stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      
  //     auto wrapper =
  //         std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  //     wrapper->setMaxIKSolutions(2);
  //     wrapper->setMinSolutionDistance(1.0);
  //     wrapper->setIKFrame("sample_holder");
  //     wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  //     wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  //     place->insert(std::move(wrapper));
  //   }

  //   // 6.2 Open Gripper
  //   {
  //     auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  //     stage->setGroup(hand_group_name);
  //     stage->setGoal("hande_open");
  //     place->insert(std::move(stage));
  //   }

  //   //6.3 Forbid Collision b/w gripper and object
  //   {
  //     auto stage =
  //         std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  //     stage->allowCollisions("sample_holder",
  //                            task.getRobotModel()
  //                                ->getJointModelGroup(hand_group_name)
  //                                ->getLinkModelNamesWithCollisionGeometry(),
  //                            false);
  //     place->insert(std::move(stage));
  //   }

  //   // 6.4 Detach Object
  //   {
  //     auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  //     stage->detachObject("sample_holder", hand_frame);
  //     place->insert(std::move(stage));
  //   }
  //   // 6.5 Place Retreat
  //   {
  //     auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //     stage->setMinMaxDistance(0.1, 0.3);
  //     stage->setIKFrame(hand_frame);
  //     stage->properties().set("marker_ns", "retreat");

  //     // Set retreat direction
  //     geometry_msgs::msg::Vector3Stamped vec;
  //     vec.header.frame_id = "map";
  //     vec.vector.x = -1.0;
  //     stage->setDirection(vec);
  //     place->insert(std::move(stage));
  //   }
  //   task.add(std::move(place));
  // }

  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("moveit_home");
  //   task.add(std::move(stage));
  // }
  return task;
}


























//   // 3. close gripper
//   auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("Close Gripper", interpolation_planner);
//   stage_close_hand->setGroup(hand_group_name);
//   stage_close_hand->setGoal("hande_closed");
//   task.add(std::move(stage_close_hand));

//   // 4. move to home
//   auto stage_move_to_home = std::make_unique<mtc::stages::MoveTo>("Move to Home", sampling_planner);
//   stage_move_to_home->setGroup(arm_group_name);
//   stage_move_to_home->setGoal("moveit_home");
//   task.add(std::move(stage_move_to_home));



// // 5. move to pick
//   {
//     auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//         "move to place",
//         mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
//                                                   { hand_group_name, sampling_planner } });
//     stage_move_to_place->setTimeout(5.0);
//     stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//     task.add(std::move(stage_move_to_place));
//   }




//   // 5. move to pick
  
//   // Sample grasp pose
//   auto stage_grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
//   stage_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
//   stage_grasp->properties().set("marker_ns", "grasp_pose");
//   stage_grasp->setPreGraspPose("hande_open");
//   stage_grasp->setObject("Cylinder_0");
//   stage_grasp->setAngleDelta(M_PI / 12);
//   stage_grasp->setMonitoredStage(current_state_ptr);  // Hook into current state
  
//   Eigen::Isometry3d grasp_frame_transform;
//   Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
//                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
//                         Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ());
//   grasp_frame_transform.linear() = q.matrix();
//   grasp_frame_transform.translation().z() = 0.14;


//     // Compute IK
//   auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage_grasp));
//   wrapper->setMaxIKSolutions(8);
//   wrapper->setMinSolutionDistance(1.0);
//   wrapper->setIKFrame(grasp_frame_transform, hand_frame);
//   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//   task.add(std::move(wrapper));

//     return task;
// }



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

  mtc_task_node->setupPlanningScene();

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

