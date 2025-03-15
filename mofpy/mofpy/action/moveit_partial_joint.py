from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
import rclpy
import rclpy.logging
from rclpy.node import Node

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveitPartialJoint(Action):

    NAME = "moveit_partial_joint"

    def __init__(self, definition, node: Node):
        super(MoveitPartialJoint, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__moveit = MoveGroupUtils.moveit
        self.__planner = MoveGroupUtils.planner
        self.__executor = MoveGroupUtils.executor

        self.__joints_targets = self.get("joints", {})

    def execute(self, named_joy=None):
        if Shared.get("move_group_disabled"):
            msg = "move_group disabled; not executing"
            rclpy.logging.get_logger("mofpy.MoveitPartialJoint").error(msg)
            return

        self.__move__()

    def __move__(self):
        rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info(
            "Moving to partial joint positions"
        )

        # 現在の関節角を取得する
        planning_scene_monitor = self.__moveit.get_planning_scene_monitor()
        with planning_scene_monitor.read_only() as scene:
            current_state = scene.current_state
            current_joint_positions = current_state.get_joint_group_positions(
                self.__planner.planning_group_name
            )

            # self.__joints_targetsに記載された関節角のみを更新する
            joints = current_state.robot_model.get_joint_model_group(
                self.__planner.planning_group_name
            ).joint_model_names

            target_joint_positions = {}
            for i in range(len(current_joint_positions)):
                # 記載されていないものはそのままの値を使用する
                if joints[i] in self.__joints_targets:
                    target_joint_positions[joints[i]] = self.__joints_targets[joints[i]]
                else:
                    target_joint_positions[joints[i]] = current_joint_positions[i]

            rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info(
                "Current joint positions: {0}".format(current_joint_positions)
            )
            rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info(
                "Target joint positions: {0}".format(target_joint_positions)
            )

            # ターゲットの関節角が指定されていない場合、または現在の関節角と同じ場合は何もしない
            if (
                not target_joint_positions
                or target_joint_positions.values() == current_joint_positions.tolist()
            ):
                rclpy.logging.get_logger("mofpy.MoveitPartialJoint").error(
                    "No target joint positions specified or target joint positions are the same as current joint positions"
                )
                return

            target_state = RobotState(current_state.robot_model)
            target_state.joint_positions = target_joint_positions
            joint_constraint = construct_joint_constraint(
                robot_state=target_state,
                joint_model_group=current_state.robot_model.get_joint_model_group(
                    self.__planner.planning_group_name
                ),
            )

            rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info(
                "Target state: {0}".format(target_state)
            )

            rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info("Setting goal state")
            self.__planner.set_goal_state(motion_plan_constraints=[joint_constraint])

        rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info("Planning")
        self.__planner.set_start_state_to_current_state()
        plan = self.__planner.plan()

        rclpy.logging.get_logger("mofpy.MoveitPartialJoint").info("Executing plan")

        if plan:
            self.__moveit.execute(plan.trajectory, controllers=[])


Action.register_preset(MoveitPartialJoint)
