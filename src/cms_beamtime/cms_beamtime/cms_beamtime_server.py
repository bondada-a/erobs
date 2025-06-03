#!/usr/bin/env python3
"""
cms_beamtime_server.py

A JSON-driven ROS2 ActionServer that uses InnerStateMachine to execute arbitrary
sequences of joint poses and gripper commands. The client must send a single string
(json_string) containing:

{
  "poses": {
    "<pose_name>": [6-joint-array],
    ...
  },
  "sequence": [
    { "type":"move", "pose_waypoints":["A","B","C"], "speed":0.5 },
    { "type":"end_effector", "device":"gripper", "action":"open" },
    { "type":"move", "pose_waypoints":["C","B","A"], "speed":0.5 },
    { "type":"end_effector", "device":"gripper", "action":"close" }
    ...
  ]
}

After each atomic step (a single-pose move or a gripper command), the server publishes
feedback = (#completed_steps / #total_steps). If the client cancels mid-run, the server
calls inner_sm.abort() and cancels the action.
"""

import threading
import time
import json
import logging
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterValue, ParameterType

from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
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

        # === PARAMETERS ===
        safe_declare('home_angles',            ParameterType.PARAMETER_DOUBLE_ARRAY)
        safe_declare('object_names',           ParameterType.PARAMETER_STRING_ARRAY)
        safe_declare('gripper_present',        ParameterType.PARAMETER_BOOL)
        safe_declare('joint_constraints.joint_name',     ParameterType.PARAMETER_STRING)
        safe_declare('joint_constraints.joint_position', ParameterType.PARAMETER_DOUBLE)

        # TF utilities
        self.tf_utils = TFUtilities(self)

        # InnerStateMachine: handles all MoveIt2 planning/execution and gripper actions
        self.inner_sm = InnerStateMachine(self, self)

        # MoveIt2 service clients (used by create_env)
        self.plan_client = self.create_client(GetMotionPlan,    '/plan_kinematic_path')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ps_client   = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Build initial planning scene
        self.apply_planning_scene(self.create_env())

        # === OBSTACLE SERVICES ===
        self.create_service(BoxObstacleMsg,      'cms_new_box_obstacle',      self.new_obstacle_cb)
        self.create_service(CylinderObstacleMsg, 'cms_new_cylinder_obstacle', self.new_obstacle_cb)
        self.create_service(UpdateObstacleMsg,   'cms_update_obstacles',      self.update_obstacles_cb)
        self.create_service(DeleteObstacleMsg,   'cms_remove_obstacle',       self.remove_obstacle_cb)

        # === Bluesky interrupt service ===
        self.create_service(BlueskyInterruptMsg, 'bluesky_interrupt',         self.bluesky_interrupt_cb)

        # === ACTION SERVER ===
        # PickPlaceControlMsg must define:
        #   string json_string
        # ---
        #   bool success
        # ---
        #   float32 status
        self._action_server = ActionServer(
            self,
            PickPlaceControlMsg,
            'cms_beamtime_action_server',
            execute_callback=self.execute_cb,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=ReentrantCallbackGroup()
        )

        # Persistent parameters
        self.gripper_present = self.get_parameter('gripper_present').value
        jc_name = self.get_parameter('joint_constraints.joint_name').value
        jc_pos  = self.get_parameter('joint_constraints.joint_position').value
        self.joint_constraints = {'name': jc_name, 'position': jc_pos}

        self.get_logger().info('cms Beamtime Server initialized.')

    # ---------------------------------------------------
    # Environment builder (unchanged)
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
    # Obstacle callbacks (unchanged)
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
    # Bluesky interrupt: delegate to inner_sm.pause()/rewind()/abort()
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
        # Always accept; validation happens in execute_cb
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested by client.')
        # Immediately abort any ongoing motion
        self.inner_sm.abort()
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        """
        JSON mode: parse goal.json_string, flatten into atomic move/gripper steps,
        and execute them in order. Publish feedback after each step.
        """
        goal = goal_handle.request
        feedback = PickPlaceControlMsg.Feedback()
        result   = PickPlaceControlMsg.Result()

        # 1) Ensure the client provided a nonempty JSON
        raw_json = getattr(goal, "json_string", "")
        if not raw_json or raw_json.strip() == "":
            self.get_logger().error("[execute_cb] Empty json_string → aborting.")
            result.success = False
            goal_handle.abort()
            return result

        # 2) Parse JSON
        try:
            data = json.loads(raw_json)
        except Exception as e:
            self.get_logger().error(f"[execute_cb] Failed to parse JSON: {e}")
            result.success = False
            goal_handle.abort()
            return result

        # 3) Extract "poses" and "sequence"
        try:
            pose_map = data["poses"]          # dict: name -> [joint-array]
            raw_sequence = data["sequence"]   # list of steps
        except KeyError as e:
            self.get_logger().error(f"[execute_cb] JSON missing key: {e}")
            result.success = False
            goal_handle.abort()
            return result

        # 4) Build a flat list of atomic steps
        atomic_steps = []
        for step in raw_sequence:
            stype = step.get("type")
            if stype == "move":
                waypoints = step.get("pose_waypoints", [])
                speed = float(step.get("speed", 1.0))
                for pname in waypoints:
                    atomic_steps.append({
                        "type": "move_single",
                        "pose_name": pname,
                        "speed": speed
                    })
            elif stype == "end_effector":
                atomic_steps.append({
                    "type":   "end_effector",
                    "device": step.get("device"),
                    "action": step.get("action")
                })
            else:
                self.get_logger().error(f"[execute_cb] Unknown step type '{stype}' in JSON.")
                result.success = False
                goal_handle.abort()
                return result

        total = len(atomic_steps)
        if total == 0:
            self.get_logger().error("[execute_cb] JSON 'sequence' is empty → aborting.")
            result.success = False
            goal_handle.abort()
            return result

        # 5) Move to HOME first
        home = self.get_parameter('home_angles').value
        self.get_logger().info(f"[execute_cb] JSON mode: moving to HOME = {home}")
        if not self.inner_sm.move_robot(home):
            self.get_logger().error("[execute_cb] JSON mode: failed to move HOME.")
            result.success = False
            goal_handle.abort()
            return result

        # 6) Loop over atomic steps
        idx = 0
        while idx < total:
            # a) Check for client cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info("[execute_cb] JSON mode: goal canceled by client.")
                self.inner_sm.abort()
                result.success = False
                goal_handle.canceled()
                return result

            step = atomic_steps[idx]
            stype = step["type"]

            try:
                if stype == "move_single":
                    pname = step["pose_name"]
                    speed = step["speed"]
                    if pname not in pose_map:
                        raise KeyError(f"Pose '{pname}' not found in JSON 'poses'")
                    joint_goal = pose_map[pname]
                    self.get_logger().info(
                        f"[execute_cb] JSON mode: Step {idx+1}/{total} → move to '{pname}' = {joint_goal} @ speed {speed}"
                    )
                    if not self.inner_sm.move_robot(joint_goal):
                        raise RuntimeError(f"move_robot failed for pose '{pname}'")

                elif stype == "end_effector":
                    device = step["device"].lower()
                    action = step["action"].lower()
                    self.get_logger().info(
                        f"[execute_cb] JSON mode: Step {idx+1}/{total} → gripper {action}"
                    )
                    if device == "gripper":
                        if action == "open":
                            if not self.inner_sm.open_gripper():
                                raise RuntimeError("gripper open failed")
                        elif action == "close":
                            if not self.inner_sm.close_gripper():
                                raise RuntimeError("gripper close failed")
                        else:
                            raise KeyError(f"Unknown gripper action '{action}'")
                    else:
                        raise KeyError(f"Unknown end_effector device '{device}'")

                else:
                    raise KeyError(f"Unhandled atomic step type '{stype}'")

            except Exception as e:
                self.get_logger().error(f"[execute_cb] JSON mode: error at step {idx}: {e}")
                self.inner_sm.abort()
                result.success = False
                goal_handle.abort()
                return result

            # b) Publish feedback
            idx += 1
            feedback.status = float(idx) / float(total)
            goal_handle.publish_feedback(feedback)

        # 7) Completed all steps successfully
        self.get_logger().info("[execute_cb] JSON mode: sequence completed successfully.")
        result.success = True
        # Ensure InnerStateMachine returns to RESTING
        self.inner_sm.set_internal_state(InternalState.RESTING)
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    server = cmsBeamtimeServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
