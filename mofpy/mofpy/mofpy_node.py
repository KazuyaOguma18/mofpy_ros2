import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .definition import Definitions
from .named_mapping import NamedMappings
from .preset_handler import PresetHandler


class Mofpy(Node):
    def __init__(self):
        super().__init__("mofpy")

        config = self.declare_parameter("config", [""]).value
        Definitions.parse(config)

        if Definitions.definitions:
            joy_bames = NamedMappings()
            handler = PresetHandler(joy_bames, self)
            handler.bind_presets(Definitions.get("presets"))

            self.joy_sub = self.create_subscription(Joy, "joy", handler.handle, 1)


def main():
    rclpy.init()
    node = Mofpy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
