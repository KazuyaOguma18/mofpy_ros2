import rclpy
import rclpy.logging

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveitNamedTarget(Action):
    """
    Control the movement towards a named target pose stored in MoveIt.

    :type __moveit: MoveitPy
    :type __planner: PlanningComponent
    :type __executor: TrajectoryExecutionManager
    """

    NAME = "moveit_named_target"

    def __init__(self, definition, node):
        super(MoveitNamedTarget, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__moveit = MoveGroupUtils.moveit
        self.__planner = MoveGroupUtils.planner
        self.__executor = MoveGroupUtils.executor
        self.__target_name = self.get_required("target_name")

    def execute(self, named_joy=None):
        if Shared.get("move_group_disabled"):
            msg = "move_group disabled; not executing: {0}".format(self.__target_name)
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error(msg)
            return

        self.__move__()

    def __move__(self):
        rclpy.logging.get_logger("mofpy.MoveitNamedTarget").info(
            "Moving to {0}".format(self.__target_name)
        )
        self.__planner.set_start_state_to_current_state()

        if self.__target_name not in self.__planner.named_target_states:
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error(
                "could not find the named target '{}'. Please select from the list '{}'".format(
                    self.__target_name, self.__planner.named_target_states
                )
            )
            return

        self.__planner.set_goal_state(configuration_name=self.__target_name)
        plan = self.__planner.plan()

        if plan:
            self.__moveit.execute(plan.trajectory, controllers=[])


Action.register_preset(MoveitNamedTarget)
