#!/usr/bin/env python3
"""
Python port of the pdf_beamtime_server C++ node.
Implements ROS2 action server, obstacle management services, and FSM-driven pick-and-place.
"""
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterValue, ParameterType
from pdf_beamtime_interfaces.action import PickPlaceControlMsg
from pdf_beamtime_interfaces.srv import (
    BoxObstacleMsg,
    CylinderObstacleMsg,
    UpdateObstacleMsg,
    DeleteObstacleMsg,
    BlueskyInterruptMsg
)

from cms_beamtime.inner_state_machine import InnerStateMachine
from cms_beamtime.tf_utilities import TFUtilities

from enum import Enum, auto

# Inline State enums (from state_enum.hpp)
class State(Enum):
    HOME = auto()
    PICKUP_APPROACH = auto()
    PICKUP = auto()
    GRASP_SUCCESS = auto()
    GRASP_FAILURE = auto()
    PICKUP_RETREAT = auto()
    PLACE_APPROACH = auto()
    PLACE = auto()
    RELEASE_SUCCESS = auto()
    RELEASE_FAILURE = auto()
    PLACE_RETREAT = auto()

class InternalState(Enum):
    RESTING = auto()
    MOVING = auto()
    PAUSED = auto()
    ABORT = auto()
    HALT = auto()
    STOP = auto()
    CLEANUP = auto()


class PdfBeamtimeServer(Node):
    def __init__(self):
        super().__init__(
            'pdf_beamtime_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        def safe_declare(name, ptype):
            if not self.has_parameter(name):
                self.declare_parameter(name, ParameterValue(type=ptype))

        safe_declare('home_angles',            ParameterType.PARAMETER_DOUBLE_ARRAY)
        safe_declare('object_names',           ParameterType.PARAMETER_STRING_ARRAY)
        safe_declare('gripper_present',        ParameterType.PARAMETER_BOOL)
        safe_declare('joint_constraints.joint_name',     ParameterType.PARAMETER_STRING)
        safe_declare('joint_constraints.joint_position', ParameterType.PARAMETER_DOUBLE)

        # TF utilities and FSM
        self.tf_utils = TFUtilities(self)
        self.inner_sm = InnerStateMachine(self, self)

        # MoveIt service clients
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ps_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Execution action client
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup()
        )

        # Build initial planning scenea
        self.apply_planning_scene(self.create_env())

        # Obstacle services
        self.create_service(BoxObstacleMsg,      'pdf_new_box_obstacle',      self.new_obstacle_cb)
        self.create_service(CylinderObstacleMsg, 'pdf_new_cylinder_obstacle', self.new_obstacle_cb)
        self.create_service(UpdateObstacleMsg,   'pdf_update_obstacles',      self.update_obstacles_cb)
        self.create_service(DeleteObstacleMsg,   'pdf_remove_obstacle',       self.remove_obstacle_cb)
        # Bluesky interrupt
        self.create_service(BlueskyInterruptMsg, 'bluesky_interrupt',         self.bluesky_interrupt_cb)

        # Action server
        self._action_server = ActionServer(
            self,
            PickPlaceControlMsg,
            'pdf_beamtime_action_server',
            execute_callback=self.execute_cb,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=ReentrantCallbackGroup()
        )

        # Read persistent params
        self.gripper_present = self.get_parameter('gripper_present').value
        jc_name = self.get_parameter('joint_constraints.joint_name').value
        jc_pos  = self.get_parameter('joint_constraints.joint_position').value
        self.joint_constraints = {'name': jc_name, 'position': jc_pos}

        self.current_state = State.HOME
        self.get_logger().info('PDF Beamtime Server initialized.')

    # Environment builder (create_env)
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
                    self.get_parameter(f'objects.{nm}.h').value,
                    self.get_parameter(f'objects.{nm}.d').value
                ]
            obj.primitives = [sp]
            pose = Pose()
            pose.position.x = self.get_parameter(f'objects.{nm}.x').value
            pose.position.y = self.get_parameter(f'objects.{nm}.y').value
            pose.position.z = self.get_parameter(f'objects.{nm}.z').value
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

    # Obstacle callbacks
    def new_obstacle_cb(self, req, resp):
        names = self.get_parameter('object_names').value
        names.append(req.name)
        self.set_parameters([rclpy.Parameter('object_names', rclpy.Parameter.Type.STRING_ARRAY, names)])
        self.declare_parameter(f'objects.{req.name}.type', req.type)
        self.declare_parameter(f'objects.{req.name}.x', req.x)
        self.declare_parameter(f'objects.{req.name}.y', req.y)
        self.declare_parameter(f'objects.{req.name}.z', req.z)
        if hasattr(req, 'w'):
            self.declare_parameter(f'objects.{req.name}.w', req.w)
            self.declare_parameter(f'objects.{req.name}.h', req.h)
            self.declare_parameter(f'objects.{req.name}.d', req.d)
        else:
            self.declare_parameter(f'objects.{req.name}.h', req.h)
            self.declare_parameter(f'objects.{req.name}.r', req.r)
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    def update_obstacles_cb(self, req, resp):
        for prop, val in zip(req.property, req.value):
            self.set_parameters([rclpy.Parameter(f'objects.{req.name}.{prop}', rclpy.Parameter.Type.DOUBLE, val)])
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    def remove_obstacle_cb(self, req, resp):
        names = [n for n in self.get_parameter('object_names').value if n != req.name]
        self.set_parameters([rclpy.Parameter('object_names', rclpy.Parameter.Type.STRING_ARRAY, names)])
        self.apply_planning_scene(self.create_env())
        resp.results = 'Success'
        return resp

    def bluesky_interrupt_cb(self, req, resp):
        # handle PAUSE, RESUME, STOP, ABORT
        st = req.interrupt_type.upper()
        if st == 'PAUSE':
            self.inner_sm.pause()
        elif st == 'RESUME':
            self.inner_sm.rewind()
        elif st in ('STOP', 'ABORT', 'HALT'):
            self.inner_sm.abort()
        resp.results = True
        return resp

    # Action callbacks
    def handle_goal(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = PickPlaceControlMsg.Feedback()
        result = PickPlaceControlMsg.Result()

        # Home position
        home = self.get_parameter('home_angles').value
        success = self.inner_sm.move_robot(home)
        if not success:
            result.success = False
            goal_handle.abort()
            return result

        self.set_current_state(State.HOME)

        # FSM loop (mirrors C++ run_fsm)
        total = len(State)
        while self.inner_sm.get_internal_state() != InternalState.STOP:
            if self.inner_sm.get_internal_state() == InternalState.PAUSED:
                time.sleep(1.0)
                continue

            if self.current_state == State.HOME:
                if self.inner_sm.move_robot(goal.pickup_approach):
                    self.set_current_state(State.PICKUP_APPROACH)
            elif self.current_state == State.PICKUP_APPROACH:
                if self.inner_sm.move_robot(goal.pickup):
                    self.set_current_state(State.PICKUP)
                if self.inner_sm.move_robot(goal.pickup):
                    self.set_current_state(State.PICKUP)
                    
            elif self.current_state == State.PICKUP:
                # close the gripper
                if self.inner_sm.close_gripper():
                    self.set_current_state(State.GRASP_SUCCESS)

            elif self.current_state == State.GRASP_SUCCESS:
                # retreat from pickup
                if self.inner_sm.move_robot(goal.pickup_approach):
                    self.set_current_state(State.PICKUP_RETREAT)

            elif self.current_state == State.PICKUP_RETREAT:
                # move into place approach
                if self.inner_sm.move_robot(goal.place_approach):
                    self.set_current_state(State.PLACE_APPROACH)

            elif self.current_state == State.PLACE_APPROACH:
                # actually place
                if self.inner_sm.move_robot(goal.place):
                    self.set_current_state(State.PLACE)

            elif self.current_state == State.PLACE:
                # open the gripper
                if self.inner_sm.open_gripper():
                    self.set_current_state(State.RELEASE_SUCCESS)

            elif self.current_state == State.RELEASE_SUCCESS:
                # back away from the place location
                if self.inner_sm.move_robot(goal.place_approach):
                    self.set_current_state(State.PLACE_RETREAT)

            elif self.current_state == State.PLACE_RETREAT:
                # done! go home
                if self.inner_sm.move_robot(home):
                    self.set_current_state(State.HOME)


            feedback.status = (list(State).index(self.current_state) + 1) / total 
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                self.reset_fsm()
                result.success = False
                goal_handle.canceled()
                return result

            # complete
            self.reset_fsm()
            result.success = True
            goal_handle.succeed()
            return result

    def set_current_state(self, state: State):
        self.get_logger().info(f"[{self.current_state.name}] Current state changed to {state.name}.")
        self.current_state = state

    def reset_fsm(self):
        self.set_current_state(State.HOME)
        self.inner_sm.set_internal_state(InternalState.RESTING)


def main():
    rclpy.init()
    server = PdfBeamtimeServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
