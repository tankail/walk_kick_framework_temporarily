#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from hightorque_msgs.msg import HightorqueJoy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMapperNode(Node):
    def __init__(self):
        super().__init__("joy_mapper_node")

        # 订阅原始手柄数据
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # 发布转换后的walk控制指令
        self.walk_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # 发布转换后的模式切换消息
        self.mode_pub = self.create_publisher(HightorqueJoy, "joy_msg", 10)

        # 定义walk映射配置
        self.walk_mappings = {
            "axis_mappings": [
                {"axis": 3, "target": "angular.z", "scale": 1.57},
                {"axis": 1, "target": "linear.x", "scale": 1.5},
                {"axis": 0, "target": "linear.y", "scale": 0.7},
                {"axis": 2, "target": "linear.z", "scale": 1.0},
            ]
        }

        # 定义mode_change映射配置
        self.mode_mappings = {
            "axis_mappings": [
                {"axis": 0, "scale": 1.0, "offset": 0.0, "target": "l_horizontal"},
                {"axis": 1, "scale": 1.0, "offset": 0.0, "target": "l_vertical"},
                {"axis": 2, "scale": 1.0, "offset": 0.0, "target": "lt"},
                {"axis": 3, "scale": 1.0, "offset": 0.0, "target": "r_horizontal"},
                {"axis": 4, "scale": 1.0, "offset": 0.0, "target": "r_vertical"},
                {"axis": 5, "scale": 1.0, "offset": 0.0, "target": "rt"},
                {"axis": 6, "scale": 1.0, "offset": 0.0, "target": "dpad_horizontal"},
                {"axis": 7, "scale": 1.0, "offset": 0.0, "target": "dpad_vertical"},
            ],
            "button_mappings": [
                {"button": 0, "target": "a"},
                {"button": 1, "target": "b"},
                {"button": 2, "target": "x"},
                {"button": 3, "target": "y"},
                {"button": 4, "target": "lb"},
                {"button": 5, "target": "rb"},
                {"button": 6, "target": "back"},
                {"button": 7, "target": "start"},
                {"button": 8, "target": "center"},
                {"button": 9, "target": "l"},
                {"button": 10, "target": "r"},
            ],
        }

        self.get_logger().info("Joy Mapper Node 已启动")

    def joy_callback(self, msg):
        try:
            # 处理walk映射 - 转换为Twist消息
            twist_msg = self.map_to_twist(msg)
            self.walk_pub.publish(twist_msg)

            # 处理mode_change映射 - 转换为HightorqueJoy消息
            joy_msg = self.map_to_hightorque_joy(msg)
            self.mode_pub.publish(joy_msg)
        except Exception as e:
            self.get_logger().error(f"处理手柄数据时出错: {str(e)}")

    def map_to_twist(self, joy_msg):
        twist = Twist()

        for mapping in self.walk_mappings["axis_mappings"]:
            axis_index = mapping["axis"]
            if axis_index < len(joy_msg.axes):
                value = joy_msg.axes[axis_index] * mapping["scale"]
                target = mapping["target"]

                # 根据目标字段设置对应的值
                if target == "linear.x":
                    twist.linear.x = value
                elif target == "linear.y":
                    twist.linear.y = value
                elif target == "linear.z":
                    twist.linear.z = value
                elif target == "angular.x":
                    twist.angular.x = value
                elif target == "angular.y":
                    twist.angular.y = value
                elif target == "angular.z":
                    twist.angular.z = value

        return twist

    def map_to_hightorque_joy(self, joy_msg):
        joy_custom = HightorqueJoy()

        # 处理轴映射
        for mapping in self.mode_mappings["axis_mappings"]:
            axis_index = mapping["axis"]
            if axis_index < len(joy_msg.axes):
                value = joy_msg.axes[axis_index] * mapping["scale"] + mapping["offset"]
                target = mapping["target"]

                # 使用setattr动态设置属性
                if hasattr(joy_custom, target):
                    setattr(joy_custom, target, float(value))

        # 处理按钮映射 - 根据HightorqueJoy消息的实际类型调整
        for mapping in self.mode_mappings["button_mappings"]:
            button_index = mapping["button"]
            if button_index < len(joy_msg.buttons):
                value = joy_msg.buttons[button_index]
                target = mapping["target"]

                # 使用setattr动态设置属性
                if hasattr(joy_custom, target):
                    # 根据目标字段的实际类型进行转换
                    field_type = type(getattr(joy_custom, target))
                    if field_type == bool:
                        setattr(joy_custom, target, bool(value))
                    elif field_type == float:
                        setattr(joy_custom, target, float(value))
                    elif field_type == int:
                        setattr(joy_custom, target, int(value))
                    else:
                        # 默认情况下，保持原始值
                        setattr(joy_custom, target, value)

        return joy_custom


def main(args=None):
    rclpy.init(args=args)
    node = JoyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
