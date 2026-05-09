# Beambot perf trace summary

- **Trace:** `/home/aditya/.ros/tracing/beambot-20260508135324`
- **Callbacks observed:** 86
- **Total callback time:** 3.10 s

## Top 25 callbacks by cumulative time (count × mean)

| Callback | Owner | Count | Mean | p95 | Total |
|--------------------------------------------------------------|-----------------------------------------------|---------|------------|------------|------------|
| void (controller_manager::ControllerManager::?)(std_msgs::m… | Subscription -- node: controller_manager, ti… | 1 | 1.16 s | 1.16 s | 1.16 s |
| ur_robot_driver::DashboardClientROS::createDashboardTrigger… | (unknown) | 1 | 645.54 ms | 645.54 ms | 645.54 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 17 | 27.49 ms | 42.77 ms | 467.38 ms |
| void (robot_state_publisher::RobotStatePublisher::?)(std::s… | Subscription -- node: robot_state_publisher,… | 13368 | 9.6 us | 23.8 us | 128.95 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 17 | 6.09 ms | 15.63 ms | 103.52 ms |
| planning_scene_monitor::CurrentStateMonitor::startStateMoni… | Subscription -- node: move_group_private_101… | 13369 | 6.9 us | 24.7 us | 92.06 ms |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 4.5 us | 10.4 us | 61.85 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 11 | 5.27 ms | 13.28 ms | 57.93 ms |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13797 | 4.1 us | 10.6 us | 56.84 ms |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 4.0 us | 9.6 us | 55.25 ms |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13797 | 4.0 us | 9.6 us | 54.78 ms |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 3.9 us | 9.0 us | 53.39 ms |
| bool(ur_controllers::GPIOController::?)(std::shared_ptr<ur_… | (unknown) | 1 | 50.15 ms | 50.15 ms | 50.15 ms |
| void (moveit_rviz_plugin::TaskDisplay::?)(std::shared_ptr<m… | Subscription -- node: rviz, tid: 126924, top… | 8 | 4.74 ms | 19.86 ms | 37.91 ms |
| planning_scene_monitor::PlanningSceneMonitor::startSceneMon… | Subscription -- node: rviz_private_137214206… | 108 | 115.3 us | 191.0 us | 12.45 ms |
| void (realtime_tools::RealtimeServerGoalHandle<control_msgs… | Timer -- node: scaled_joint_trajectory_contr… | 424 | 12.5 us | 49.7 us | 5.29 ms |
| void (diagnostic_updater::Updater::?)() | Timer -- node: controller_manager, tid: 1269… | 37 | 140.0 us | 197.3 us | 5.18 ms |
| void (zivid_camera::ZividCamera::?)() | Timer -- node: zivid_camera, tid: 126601, pe… | 4 | 1.20 ms | 1.23 ms | 4.79 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 39 | 82.6 us | 174.6 us | 3.22 ms |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: rviz, tid: 126924, top… | 994 | 2.7 us | 5.0 us | 2.73 ms |
| planning_scene_monitor::PlanningSceneMonitor::startStateMon… | Timer -- node: move_group_private_1013698520… | 1209 | 1.8 us | 2.5 us | 2.20 ms |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_mtc, tid: 1265… | 995 | 1.6 us | 3.3 us | 1.58 ms |
| void (realtime_tools::RealtimeServerGoalHandle<control_msgs… | Timer -- node: scaled_joint_trajectory_contr… | 54 | 29.2 us | 61.3 us | 1.58 ms |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: force_mode_controller,… | 783 | 2.0 us | 1.9 us | 1.57 ms |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_mtc, tid: 1265… | 1031 | 1.5 us | 3.3 us | 1.54 ms |

## Top 25 callbacks by p95 latency (>=3 samples)

| Callback | Owner | Count | p50 | p95 | Max |
|--------------------------------------------------------------|-----------------------------------------------|---------|------------|------------|------------|
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 17 | 24.57 ms | 42.77 ms | 45.62 ms |
| void (moveit_rviz_plugin::TaskDisplay::?)(std::shared_ptr<m… | Subscription -- node: rviz, tid: 126924, top… | 8 | 137.2 us | 19.86 ms | 22.64 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 17 | 4.50 ms | 15.63 ms | 16.15 ms |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 11 | 3.19 ms | 13.28 ms | 14.91 ms |
| void (zivid_camera::ZividCamera::?)() | Timer -- node: zivid_camera, tid: 126601, pe… | 4 | 1.20 ms | 1.23 ms | 1.23 ms |
| void (diagnostic_updater::Updater::?)() | Timer -- node: controller_manager, tid: 1269… | 37 | 138.9 us | 197.3 us | 245.7 us |
| planning_scene_monitor::PlanningSceneMonitor::startSceneMon… | Subscription -- node: rviz_private_137214206… | 108 | 108.2 us | 191.0 us | 289.6 us |
| void (controller_manager::ControllerManager::?)(std::shared… | (unknown) | 39 | 85.7 us | 174.6 us | 189.7 us |
| planning_scene_monitor::PlanningSceneMonitor::providePlanni… | (unknown) | 3 | 87.9 us | 94.7 us | 95.4 us |
| void (realtime_tools::RealtimeServerGoalHandle<control_msgs… | Timer -- node: scaled_joint_trajectory_contr… | 54 | 32.1 us | 61.3 us | 129.3 us |
| void (realtime_tools::RealtimeServerGoalHandle<control_msgs… | Timer -- node: scaled_joint_trajectory_contr… | 424 | 5.4 us | 49.7 us | 79.4 us |
| planning_scene_monitor::CurrentStateMonitor::startStateMoni… | Subscription -- node: move_group_private_101… | 13369 | 4.4 us | 24.7 us | 252.0 us |
| void (robot_state_publisher::RobotStatePublisher::?)(std::s… | Subscription -- node: robot_state_publisher,… | 13368 | 4.7 us | 23.8 us | 234.8 us |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13797 | 2.8 us | 10.6 us | 79.6 us |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 3.5 us | 10.4 us | 134.3 us |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 3.0 us | 9.6 us | 154.1 us |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13797 | 3.0 us | 9.6 us | 146.3 us |
| void (tf2_ros::TransformListener::?)(std::shared_ptr<tf2_ms… | Subscription -- node: transform_listener_imp… | 13796 | 3.0 us | 9.0 us | 143.6 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_moveto_server,… | 35 | 2.8 us | 7.0 us | 8.4 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_moveto_server,… | 91 | 1.1 us | 5.9 us | 8.2 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_moveto_server,… | 5 | 0.4 us | 5.1 us | 6.2 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_moveto_server,… | 5 | 0.5 us | 5.0 us | 6.1 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: rviz, tid: 126924, top… | 994 | 2.5 us | 5.0 us | 25.8 us |
| rclcpp::TimeSource::NodeState::attachNode(std::shared_ptr<r… | Subscription -- node: beambot_moveto_server,… | 5 | 0.6 us | 4.4 us | 5.3 us |
| void (robot_state_publisher::RobotStatePublisher::?)(std::s… | Subscription -- node: robot_state_publisher,… | 1000 | 0.8 us | 4.0 us | 22.8 us |

## Top 25 owners by cumulative time

| Owner (node / topic / service) | # cbs | Invocations | Total |
|---------------------------------------------------------|---------|--------------|------------|
| (unknown) | 12 | 1098 | 1.33 s |
| Subscription -- node: controller_manager, tid: 126913,… | 1 | 1 | 1.16 s |
| Subscription -- node: robot_state_publisher, tid: 1269… | 1 | 13368 | 128.95 ms |
| Subscription -- node: move_group_private_1013698520878… | 1 | 13369 | 92.06 ms |
| Subscription -- node: transform_listener_impl_5c32021a… | 1 | 13796 | 61.85 ms |
| Subscription -- node: transform_listener_impl_566e4e7f… | 1 | 13797 | 56.84 ms |
| Subscription -- node: transform_listener_impl_5df9d9cc… | 1 | 13796 | 55.25 ms |
| Subscription -- node: transform_listener_impl_7ccbac36… | 1 | 13797 | 54.78 ms |
| Subscription -- node: transform_listener_impl_5df9d9bb… | 1 | 13796 | 53.39 ms |
| Subscription -- node: rviz, tid: 126924, topic: /descr… | 1 | 8 | 37.91 ms |
| Subscription -- node: rviz_private_137214206667888, ti… | 1 | 108 | 12.45 ms |
| Timer -- node: scaled_joint_trajectory_controller, tid… | 1 | 424 | 5.29 ms |
| Timer -- node: controller_manager, tid: 126913, period… | 1 | 37 | 5.18 ms |
| Timer -- node: zivid_camera, tid: 126601, period: 1000… | 1 | 4 | 4.79 ms |
| Subscription -- node: rviz, tid: 126924, topic: /param… | 1 | 994 | 2.73 ms |
| Timer -- node: move_group_private_101369852087808, tid… | 1 | 1209 | 2.20 ms |
| Subscription -- node: robot_state_publisher, tid: 1269… | 2 | 2019 | 2.18 ms |
| Subscription -- node: beambot_mtc, tid: 126597, topic:… | 1 | 995 | 1.58 ms |
| Timer -- node: scaled_joint_trajectory_controller, tid… | 1 | 54 | 1.58 ms |
| Subscription -- node: force_mode_controller, tid: 1270… | 1 | 783 | 1.57 ms |
| Subscription -- node: beambot_mtc, tid: 126596, topic:… | 1 | 1031 | 1.54 ms |
| Subscription -- node: beambot_mtc, tid: 126600, topic:… | 1 | 992 | 1.40 ms |
| Subscription -- node: beambot_mtc, tid: 126603, topic:… | 1 | 995 | 1.36 ms |
| Subscription -- node: trajectory_until_node, tid: 1269… | 1 | 1035 | 1.35 ms |
| Subscription -- node: beambot_mtc, tid: 126602, topic:… | 1 | 994 | 1.32 ms |

---
_For publish/receive latency and timeline, run `ros2 run tracetools_analysis auto /home/aditya/.ros/tracing/beambot-20260508135324`._
