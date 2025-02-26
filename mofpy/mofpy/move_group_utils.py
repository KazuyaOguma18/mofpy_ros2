from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit.planning import TrajectoryExecutionManager
from rclpy.node import Node


class MoveGroupUtils:
    moveit: MoveItPy = None
    planner: PlanningComponent = None
    executor: TrajectoryExecutionManager = None
    namespace: str = None
    servo_node_name: str = None

    def __init__(self):
        pass

    @staticmethod
    def connect(node: Node):
        p = "move_group"
        planning_group = node.declare_parameter(p + ".planning_group", "arm").value
        namespace = node.declare_parameter(p + ".namespace", "").value
        servo_node_name = node.declare_parameter(p + ".servo_node_name", "servo_node").value
        use_global_joint_states = node.declare_parameter(
            p + ".use_global_joint_states", False
        ).value
        MoveGroupUtils.namespace = namespace
        MoveGroupUtils.servo_node_name = namespace + "/" + servo_node_name
        remappings = [
            ("joint_states", "/joint_states"),
        ]

        try:
            MoveGroupUtils.moveit = MoveItPy(
                node_name=node.get_name() + "_moveit",
                name_space=namespace,
                remappings=remappings if use_global_joint_states else None,
            )
        except Exception as e:
            node.get_logger().error(f"Failed to initialize MoveItPy: {str(e)}")
            return False

        # Sometimes, MoveGroupCommander fails to initialize.
        # Try several times and then give up.
        attempt = 0
        while attempt <= 3:
            try:
                MoveGroupUtils.planner = MoveGroupUtils.moveit.get_planning_component(
                    planning_group
                )
                MoveGroupUtils.executor = MoveGroupUtils.moveit.get_trajectory_execution_manager()
                return True
            except Exception as e:
                node.get_logger().error(f"Failed to initialize MoveIt planning component: {str(e)}")
                attempt += 1
        return False
