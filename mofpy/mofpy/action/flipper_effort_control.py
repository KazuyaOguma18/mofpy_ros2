import re
import threading
import uuid

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.executors import SingleThreadedExecutor
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .action import Action
from ..shared import Shared


class FlipperEffortControl(Action):
    NAME = "flipper_effort_control"

    def __init__(self, definition, node: Node):
        super(FlipperEffortControl, self).__init__(definition, node)

        self.__namespace = self.get_required("namespace")
        self.controller_name = self.get_required("controller_name")

        client_node = Node(
            node.get_name() + "_flipper_eff_control_" + str(uuid.uuid4()).replace("-", "")
        )
        self.__list_controller_client = client_node.create_client(
            ListControllers, self.__namespace + "/controller_manager/list_controllers"
        )

        self.__switch_controller_client = client_node.create_client(
            SwitchController, self.__namespace + "/controller_manager/switch_controller"
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(client_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.__flipper_pub = node.create_publisher(
            Float64MultiArray, self.__namespace + "/" + self.controller_name + "/commands", 10
        )

        self.__joint_names = None
        self.__is_first = True

        self.__efforts = self.__get_efforts__()

    def execute(self, named_joy=None):

        if self.__is_first:
            self.__is_first = False
            self.__flipper_init__()

        if Shared.get("flipper_command_type") != "flipper_effort_control":
            if not self.__flipper_init__():
                rclpy.logging.get_logger("mofpy.FlipperEffortControl").error(
                    "Failed to initialize flipper effort control"
                )
                return

        # フリッパの関節角度を取得
        joint_efforts = []
        for joint_name in self.__joint_names:
            if joint_name in self.__efforts:
                joint_efforts.append(self.__efforts[joint_name])

        # 制御量と関節角名の配列サイズが異なる場合はエラー
        if len(joint_efforts) != len(self.__joint_names):
            rclpy.logging.get_logger("mofpy.FlipperEffortControl").error("Invalid joint_efforts")
            return

        self.__pub_flipper__(joint_efforts)

    def __flipper_init__(self):

        # コントローラの一覧を取得
        controllers = self.__get_controller_list__()

        # 該当のコントローラが存在するか確認
        flipper_controller: ControllerState = None
        for controller in controllers:
            if self.controller_name == controller.name:
                flipper_controller = controller
                break

        if flipper_controller is None:
            rclpy.logging.get_logger("mofpy.FlipperEffortControl").error("Controller not found")
            return False

        if flipper_controller.state != "active":
            # コントローラを有効化
            success = self.__switch_controller__(flipper_controller, controllers)
            if not success:
                rclpy.logging.get_logger("mofpy.FlipperEffortControl").error(
                    "Failed to switch controller"
                )
                return False

        Shared.update("flipper_command_type", "flipper_effort_control")

        # self.__joint_namesが存在しない場合は新規で登録
        if self.__joint_names is None:
            self.__joint_names = []
            # joint_namesをコントローラ一覧から抽出
            for interface in flipper_controller.required_command_interfaces:
                self.__joint_names.append(re.sub(r"/effort$", "", interface))

        return True

    def __get_controller_list__(self):
        # コントローラの一覧を取得
        req = ListControllers.Request()
        self.lc_future = self.__list_controller_client.call_async(req)
        self.executor.spin_until_future_complete(future=self.lc_future, timeout_sec=1)

        res: ListControllers.Response = self.lc_future.result()
        if res is None:
            rclpy.logging.get_logger("mofpy.FlipperEffortControl").error(
                "Failed to get controller list"
            )
            return None

        return res.controller

    def __switch_controller__(
        self, start_controller: ControllerState, controllers: list[ControllerState]
    ):
        # start_controllerと同じjoint_namesを持つcontrollersを探す
        # そのコントローラを無効化
        stop_controllers = []
        for controller in controllers:
            exist_interfaces = [
                interface.rsplit("/", 1)[0] for interface in controller.required_command_interfaces
            ]
            start_interfaces = [
                interface.rsplit("/", 1)[0]
                for interface in start_controller.required_command_interfaces
            ]
            if exist_interfaces == start_interfaces and controller.name != start_controller.name:
                stop_controllers.append(controller.name)

        req = SwitchController.Request()
        req.activate_controllers = [start_controller.name]
        req.deactivate_controllers = stop_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT

        self.sc_future = self.__switch_controller_client.call_async(req)
        self.executor.spin_until_future_complete(future=self.sc_future, timeout_sec=1)

        res: SwitchController.Response = self.sc_future.result()
        if res is None:
            rclpy.logging.get_logger("mofpy.FlipperEffortControl").error(
                "Failed to switch controller"
            )
            return False

        return res.ok

    def __get_efforts__(self):
        params = self.get("joints", {})
        return params

    def __pub_flipper__(self, joint_efforts):
        msg = Float64MultiArray()
        msg.data = joint_efforts
        self.__flipper_pub.publish(msg)


Action.register_preset(FlipperEffortControl)
