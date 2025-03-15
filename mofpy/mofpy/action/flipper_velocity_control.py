import re
import threading
import uuid

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from rclpy.executors import SingleThreadedExecutor
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .action import Action
from ..shared import Shared


# サブクローラ(フリッパ)の角速度制御
# FL : 左前フリッパ
# RL : 左後フリッパ
# FR : 右前フリッパ
# RR : 右後ろフリッパ
class FlipperVelocityControl(Action):
    NAME = "flipper_velocity_control"

    def __init__(self, definition, node: Node):
        super(FlipperVelocityControl, self).__init__(definition, node)

        self.__namespace = self.get_required("namespace")
        self.controller_name = self.get_required("controller_name")

        client_node = Node(
            node.get_name() + "_flipper_vel_control_" + str(uuid.uuid4()).replace("-", "")
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

        self.__mapping = self.__get_mapping__()

        self.__scale = self.get("scale", 0.1)

        self.__is_first = True

        self.__published_zero = False

    def execute(self, named_joy=None):

        if self.__is_first:
            self.__is_first = False
            self.__flipper_init__()

        # 独立制御モードか否かを判定
        mode = Shared.get("flipper_control_mode", "synchronous")

        # フリッパの関節角速度を設定
        buttons = named_joy["buttons"]
        joint_velocities = []

        # 統合制御モードの場合(前フリッパ，後フリッパを同時制御)
        if mode == "synchronous":
            front_flippers = self.__mapping[mode]["front_flippers"]
            rear_flippers = self.__mapping[mode]["rear_flippers"]
            if front_flippers is None or rear_flippers is None:
                rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                    "Front or rear flippers are empty"
                )
                return
            for joint_name in self.__joint_names:
                if joint_name in front_flippers:
                    v = 0
                    if buttons[self.__mapping[mode]["front"]["raise"]].value:
                        v = 1
                    if buttons[self.__mapping[mode]["front"]["lower"]].value:
                        v = -1
                    v = v * self.__scale
                    joint_velocities.append(v)
                elif joint_name in rear_flippers:
                    v = 0
                    if buttons[self.__mapping[mode]["rear"]["raise"]].value:
                        v = 1
                    if buttons[self.__mapping[mode]["rear"]["lower"]].value:
                        v = -1
                    v = v * self.__scale
                    joint_velocities.append(v)

        # 独立制御モードの場合
        if mode == "independent":
            joint_mapping = self.__mapping[mode]["joints"]
            if list(joint_mapping.keys()) != self.__joint_names:
                rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                    "Joint names are not same between controller joints and mapping joints\n {} vs {}".format(
                        self.__joint_names, list(joint_mapping.keys())
                    )
                )
                return
            is_raise = buttons[self.__mapping[mode]["raise"]].value
            is_lower = buttons[self.__mapping[mode]["lower"]].value
            if is_raise:
                direction = 1
            elif is_lower:
                direction = -1
            else:
                direction = 0

            for joint_name in self.__joint_names:
                if joint_mapping[joint_name] is None:
                    rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                        "Joint not found {}".format(joint_name)
                    )
                    return
                v = 1 if buttons[joint_mapping[joint_name]].value else 0
                v = v * direction * self.__scale
                joint_velocities.append(v)

        # 制御量と関節角名の配列サイズが異なる場合はエラー
        if len(joint_velocities) != len(self.__joint_names):
            rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                "Failed to set joint velocities {} vs {}".format(
                    len(joint_velocities), len(self.__joint_names)
                )
            )
            return

        # 全ての角速度が0の場合はトピックは配信しない
        all_zero = all(v == 0 for v in joint_velocities)
        if all_zero and self.__published_zero:
            return

        if Shared.get("flipper_command_type") != "flipper_velocity_control":
            if not self.__flipper_init__():
                rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                    "Failed to initialize flipper velocity control"
                )
                return

        # if all_zero:
        #     joint_velocities = [1e-04 for _ in joint_velocities]

        # フリッパの関節角を配信
        self.__pub_flipper__(joint_velocities)
        self.__published_zero = all_zero

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
            rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error("Controller not found")
            return False

        if flipper_controller.state != "active":
            # コントローラを有効化
            success = self.__switch_controller__(flipper_controller, controllers)
            if not success:
                rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                    "Failed to switch controller"
                )
                return False

        Shared.update("flipper_command_type", "flipper_velocity_control")

        # self.__joint_namesが存在しない場合は新規で登録
        if self.__joint_names is None:
            self.__joint_names = []
            # joint_namesをコントローラ一覧から抽出
            for interface in flipper_controller.required_command_interfaces:
                self.__joint_names.append(re.sub(r"/velocity$", "", interface))

        return True

    def __get_controller_list__(self):
        # コントローラの一覧を取得
        req = ListControllers.Request()
        self.lc_future = self.__list_controller_client.call_async(req)
        self.executor.spin_until_future_complete(future=self.lc_future, timeout_sec=1)

        res: ListControllers.Response = self.lc_future.result()
        if res is None:
            rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                "Failed to get controller list"
            )
            return None

        return res.controller

    def __switch_controller__(
        self, start_controller: ControllerState, controllers: list[ControllerState]
    ):
        # start_controllerと同じjoint_namesを持つのcontrollersを探す
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
            rclpy.logging.get_logger("mofpy.FlipperVelocityControl").error(
                "Failed to switch controller"
            )
            return False

        return res.ok

    # マッピングを取得
    def __get_mapping__(self):
        modes = ["synchronous", "independent"]
        directions = ["raise", "lower"]
        sides = ["front", "rear"]
        params = self.get("mapping", {})
        mapping = {}

        for mode in modes:
            mapping[mode] = {}
            for side in sides:
                mapping[mode][side] = {}
                for direction in directions:
                    if mode == "synchronous":
                        mapping[mode][side][direction] = (
                            params.get(mode, {}).get(side, {}).get(direction, None)
                        )
                    elif mode == "independent":
                        mapping[mode][direction] = params.get(mode, {}).get(direction, None)
            if mode == "synchronous":
                mapping[mode]["front_flippers"] = params.get(mode, {}).get("front_flippers", None)
                mapping[mode]["rear_flippers"] = params.get(mode, {}).get("rear_flippers", None)
            elif mode == "independent":
                mapping[mode]["joints"] = params.get(mode, {}).get("joints", None)

        return mapping

    def __pub_flipper__(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities
        self.__flipper_pub.publish(msg)


Action.register_preset(FlipperVelocityControl)
