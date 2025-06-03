#!/usr/bin/env python3
import json
import logging
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterValue, ParameterType

from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from cms_beamtime.inner_state_machine import InnerStateMachine
from cms_beamtime.tf_utilities import TFUtilities

from cms_beamtime_interfaces.action import PickPlaceControlMsg
from cms_beamtime_interfaces.srv import (
    BoxObstacleMsg,
    CylinderObstacleMsg,
    UpdateObstacleMsg,
    DeleteObstacleMsg,
    BlueskyInterruptMsg
)
from enum import Enum, auto


class InternalState(Enum):
    RESTING = auto()
    MOVING = auto()
    PAUSED = auto()
    ABORT = auto()
    HALT = auto()
    STOP = auto()
    CLEANUP = auto()


logger = logging.getLogger("cms_beamtime_server")
logger.setLevel(logging.INFO)


class cmsBeamtimeServer(Node):
    def __init__(self):
        super().__init__(
            'cms_beamtime_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        def safe_declare(name, ptype):
            if not self.has_parameter(name):
                self.declare_parameter(name, ParameterValue(type=ptype))

        safe_declare('home_angles',            ParameterType.PARAMETER_DOUBLE_ARRAY)
        safe_declare('object_names',           ParameterType.PARAMETER_STRING_ARRAY)
        safe_declare('joint_constraints.joint_name',     ParameterType.PARAMETER_STRING)
        safe_declare('joint_constraints.joint_position', ParameterType.PARAMETER_DOUBLE)
        safe_declare('joint_constraints.upper_limit',    ParameterType.PARAMETER_DOUBLE)
        safe_declare('joint_constraints.lower_limit',    ParameterType.PARAMETER_DOUBLE)

        self.tf_utils = TFUtilities(self)
        self.inner_sm = InnerStateMachine(self, self)

        # MoveGroupInterface for path constraints
        try:
            from moveit_py.move_group_interface import MoveGroupInterface
        except ImportError:
            from moveit2 import MoveGroupInterface  # type: ignore

        self.move_group_interface = MoveGroupInterface(self, 'ur_arm')

        self.plan_client = self.create_client(GetMotionPlan,    '/plan_kinematic_path')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ps_client   = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        self.apply_planning_scene(self.create_env())

        # === OBSTACLE SERVICES ===
        self.create_service(BoxObstacleMsg,      'cms_new_box_obstacle',      self.new_obstacle_cb)
        self.create_service(CylinderObstacleMsg, 'cms_new_cylinder_obstacle', self.new_obstacle_cb)
        self.create_service(UpdateObstacleMsg,   'cms_update_obstacles',      self.update_obstacles_cb)
        self.create_service(DeleteObstacleMsg,   'cms_remove_obstacle',       self.remove_obstacle_cb)

        # === Bluesky interrupt service ===
        self.create_service(BlueskyInterruptMsg, 'bluesky_interrupt',         self.bluesky_interrupt_cb)

        # === ACTION SERVER ===
        self._action_server = ActionServer(
            self,
            PickPlaceControlMsg,
            'cms_beamtime_action_server',
            execute_callback=self.execute_cb,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=ReentrantCallbackGroup()
        )
        ## Joint constraints for wrist mov
        jc_name = self.get_parameter('joint_constraints.joint_name').value
        jc_pos  = self.get_parameter('joint_constraints.joint_position').value
        jc_upper = self.get_parameter('joint_constraints.upper_limit').value
        jc_lower = self.get_parameter('joint_constraints.lower_limit').value

        jc = JointConstraint()
        jc.joint_name = jc_name
        jc.position = jc_pos
        jc.tolerance_above = jc_upper - jc.position
        jc.tolerance_below = jc.position - jc_lower
        jc.weight = 1.0

        constraints = Constraints()
        constraints.joint_constraints = [jc]

        self.joint_constraints = constraints
        # Cleanup strategy: 'step' follows the path back, 'home' goes straight home
        if not self.has_parameter('cleanup_mode'):
            self.declare_parameter('cleanup_mode', 'step')
        self.cleanup_mode = self.get_parameter('cleanup_mode').value

        # Track the currently executing external state
        self.current_state = None
        # Record executed poses for cleanup
        self.executed_moves: List[str] = []
        # Track last gripper action ('open' or 'close')
        self.last_gripper_action = 'open'

        self.get_logger().info('cms Beamtime Server initialized.')

    # ---------------------------------------------------
    # Environment builder 
    def create_env(self) -> List[CollisionObject]:
        names = self.get_parameter('object_names').value
        objs: List[CollisionObject] = []
        for nm in names:
            obj = CollisionObject()
            obj.id = nm
            obj.header.frame_id = self.tf_utils.world_frame
            typ = self.get_parameter(f'objects.{nm}.type').value
            if typ == 'CYLINDER':
                sp = SolidPrimitive()
                sp.type = SolidPrimitive.CYLINDER
                sp.dimensions = [
                    self.get_parameter(f'objects.{nm}.h').value,
                    self.get_parameter(f'objects.{nm}.r').value
                ]
            else:
                sp = SolidPrimitive()
                sp.type = SolidPrimitive.BOX
                sp.dimensions = [
                    self.get_parameter(f'objects.{nm}.w').value,
                    self.get_parameter(f'objects.{nm}.d').value,
                    self.get_parameter(f'objects.{nm}.h').value
                ]
            obj.primitives = [sp]

            pose = Pose()
            pose.position.x = self.get_parameter(f'objects.{nm}.x').value
            pose.position.y = self.get_parameter(f'objects.{nm}.y').value
            pose.position.z = self.get_parameter(f'objects.{nm}.z').value
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD
            objs.append(obj)
        return objs

    def apply_planning_scene(self, objs: List[CollisionObject]):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world.collision_objects = objs
        fut = self.ps_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

    # ---------------------------------------------------
    # Obstacle callbacks 
    def new_obstacle_cb(self, req, resp):
        names = self.get_parameter('object_names').value
        names.append(req.name)
        self.set_parameters([rclpy.Parameter('object_names', rclpy.Parameter.Type.STRING_ARRAY, names)])
        self.declare_parameter(f'objects.{req.name}.type', req.type)
        self.declare_parameter(f'objects.{req.name}.x', req.x)
        self.declare_parameter(f'objects.{req.name}.y', req.y)
        self.declare_parameter(f'objects.{req.name}.z', req.z)
        if hasattr(req, 'w'):  # BOX
            self.declare_parameter(f'objects.{req.name}.w', req.w)
            self.declare_parameter(f'objects.{req.name}.h', req.h)
            self.declare_parameter(f'objects.{req.name}.d', req.d)
        else:  # CYLINDER
            self.declare_parameter(f'objects.{req.name}.h', req.h)
            self.declare_parameter(f'objects.{req.name}.r', req.r)
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    def update_obstacles_cb(self, req, resp):
        for prop, val in zip(req.property, req.value):
            self.set_parameters([
                rclpy.Parameter(f'objects.{req.name}.{prop}', rclpy.Parameter.Type.DOUBLE, val)
            ])
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    def remove_obstacle_cb(self, req, resp):
        names = [n for n in self.get_parameter('object_names').value if n != req.name]
        self.set_parameters([rclpy.Parameter('object_names', rclpy.Parameter.Type.STRING_ARRAY, names)])
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    # ---------------------------------------------------
    # Bluesky interrupt:
    def bluesky_interrupt_cb(self, req, resp):
        st = req.interrupt_type.upper()
        if st == 'PAUSE':
            self.get_logger().info("Bluesky requested PAUSE.")
            self.inner_sm.pause()
        elif st == 'RESUME':
            self.get_logger().info("Bluesky requested RESUME.")
            self.inner_sm.rewind()
        elif st in ('STOP', 'ABORT', 'HALT'):
            self.get_logger().info(f"Bluesky requested {st}. Aborting.")
            self.inner_sm.abort()
        else:
            self.get_logger().warn(f"Unknown bluesky interrupt: {st}")
        resp.results = True
        return resp

    # ---------------------------------------------------
    # Action server callbacks
    def handle_goal(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested by client.')
        self.inner_sm.abort()
        return CancelResponse.ACCEPT
   # ---------------------------------------------------
    def execute_cleanup(self, mode: str | None = None):
        """Attempt to return the robot to a safe configuration.

        Parameters
        ----------
        mode : str, optional
            Either ``"step"`` to unwind the executed poses back to ``home`` or
            ``"home"`` to move directly to ``home``.  If ``None`` the value of
            the ``cleanup_mode`` parameter is used.
        """

        if mode is None:
            mode = self.cleanup_mode

        pose_map = getattr(self, "pose_map", {})
        home = self.get_parameter('home_angles').value
        state_name = self.current_state.name if self.current_state else "UNKNOWN"
        self.get_logger().info(
            f"[Cleanup] Mode={mode} handling state {state_name}")

        self.inner_sm.set_internal_state(InternalState.CLEANUP)

        if mode == 'home':
            self.move_group_interface.set_path_constraints(self.joint_constraints)
            self.inner_sm.move_robot(home)
            self.move_group_interface.clear_path_constraints()
            if self.last_gripper_action == 'close':
                self.inner_sm.open_gripper()
                self.last_gripper_action = 'open'
            self.inner_sm.set_internal_state(InternalState.RESTING)
            self.executed_moves = []
            return

        executed = getattr(self, 'executed_moves', [])
        for pname in reversed(executed):
            if pname in pose_map:
                self.move_group_interface.set_path_constraints(self.joint_constraints)
                self.inner_sm.move_robot(pose_map[pname])
                self.move_group_interface.clear_path_constraints()

        if self.last_gripper_action == 'close':
            self.inner_sm.open_gripper()
            self.last_gripper_action = 'open'

        self.move_group_interface.set_path_constraints(self.joint_constraints)
        self.inner_sm.move_robot(home)
        self.move_group_interface.clear_path_constraints()
        self.inner_sm.set_internal_state(InternalState.RESTING)
        self.executed_moves = []

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = PickPlaceControlMsg.Feedback()
        result   = PickPlaceControlMsg.Result()

        raw_json = getattr(goal, "json_string", "")
        if not raw_json or raw_json.strip() == "":
            self.get_logger().error("Action Server : Empty json_string → aborting.")
            result.success = False
            goal_handle.abort()
            return result

        try:
            data = json.loads(raw_json)
        except Exception as e:
            self.get_logger().error(f"Action Server : Failed to parse JSON: {e}")
            result.success = False
            goal_handle.abort()
            return result

        try:
            pose_map = data["poses"]          # dict: name -> [joint-array]
            raw_sequence = data["sequence"]   # list of steps
        except KeyError as e:
            self.get_logger().error(f"Action Server : JSON missing key: {e}")
            result.success = False
            goal_handle.abort()
            return result

        self.pose_map = pose_map
        ExternalState = Enum('ExternalState', {k.upper(): auto() for k in pose_map.keys()})
        self.executed_moves = []
        self.last_gripper_action = 'open'

        sequence_steps = []
        for step in raw_sequence:
            stype = step.get("type")
            if stype == "move":
                waypoints = step.get("pose_waypoints", [])
                for pname in waypoints:
                    sequence_steps.append({
                        "type": "move_single",
                        "pose_name": pname,
                    })
            elif stype == "end_effector":
                sequence_steps.append({
                    "type":   "end_effector",
                    "device": step.get("device"),
                    "action": step.get("action")
                })
            else:
                self.get_logger().error(f"[Action Server] Unknown step type '{stype}' in JSON.")
                result.success = False
                goal_handle.abort()
                return result

        total = len(sequence_steps)
        if total == 0:
            self.get_logger().error("[Action Server] JSON 'sequence' is empty → aborting.")
            result.success = False
            goal_handle.abort()
            return result

        # Move to HOME first
        home = self.get_parameter('home_angles').value
        self.get_logger().info(f"[Action Server]  moving to HOME = {home}")
        self.move_group_interface.set_path_constraints(self.joint_constraints)
        success = self.inner_sm.move_robot(home)
        self.move_group_interface.clear_path_constraints()
        if not success:
            self.get_logger().error("[Action Server]  failed to move HOME.")
            self.execute_cleanup()

            result.success = False
            goal_handle.abort()
            return result

        # Loop over sequence steps
        idx = 0
        while idx < total:

            if goal_handle.is_cancel_requested:
                self.get_logger().info("[Action Server]  goal canceled by client.")
                self.inner_sm.abort()
                self.execute_cleanup()
                result.success = False
                goal_handle.canceled()
                return result

            step = sequence_steps[idx]
            stype = step["type"]

            try:
                if stype == "move_single":
                    pname = step["pose_name"]
                    
                    if pname not in pose_map:
                        raise KeyError(f"Pose '{pname}' not found in JSON 'poses'")
                    joint_goal = pose_map[pname]
                    self.get_logger().info(
                        f"[Action Server]  Step {idx+1}/{total} → move to '{pname}' = {joint_goal}"
                    )
                    
                    self.current_state = ExternalState[pname.upper()]

                    self.move_group_interface.set_path_constraints(self.joint_constraints)
                    move_success = self.inner_sm.move_robot(joint_goal)
                    self.move_group_interface.clear_path_constraints()
                    if not move_success:
                        raise RuntimeError(f"move_robot failed for pose '{pname}'")
                    self.executed_moves.append(pname)


                elif stype == "end_effector":
                    device = step["device"].lower()
                    action = step["action"].lower()
                    self.get_logger().info(
                        f"[Action Server]  Step {idx+1}/{total} → gripper {action}"
                    )
                    if device == "gripper":
                        if action == "open":
                            if not self.inner_sm.open_gripper():
                                raise RuntimeError("gripper open failed")
                            self.last_gripper_action = 'open'
                            if hasattr(ExternalState, "PLACE_RETREAT"):
                                self.current_state = ExternalState.PLACE_RETREAT
                        elif action == "close":
                            if not self.inner_sm.close_gripper():
                                raise RuntimeError("gripper close failed")
                            self.last_gripper_action = 'close'
                            if hasattr(ExternalState, "GRASP_SUCCESS"):
                                self.current_state = ExternalState.GRASP_SUCCESS
                        else:
                            raise KeyError(f"Unknown gripper action '{action}'")
                    else:
                        raise KeyError(f"Unknown end_effector device '{device}'")

                else:
                    raise KeyError(f"Unhandled atomic step type '{stype}'")

            except Exception as e:
                self.get_logger().error(f"[Action Server] error at step {idx}: {e}")
                self.inner_sm.abort()
                self.execute_cleanup()

                result.success = False
                goal_handle.abort()
                return result

            idx += 1
            feedback.status = float(idx) / float(total)
            goal_handle.publish_feedback(feedback)

        self.get_logger().info("[Action Server]  sequence completed successfully.")
        result.success = True
        self.inner_sm.set_internal_state(InternalState.RESTING)
        self.executed_moves = []
        self.last_gripper_action = 'open'
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    server = cmsBeamtimeServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()