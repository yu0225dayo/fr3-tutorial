#!/usr/bin/env python3
"""偽の力覚センサに流している力を、RViz の矢印で表示する。
力 = 赤い矢印（1N あたり 1.5cm）、トルク = 青い矢印（1Nm あたり 5cm）。どちらも力覚センサの座標系（wrist3_link）の原点から出る。
力が 0 になる、または指令が止まると矢印は消える。"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

FRAME = "wrist3_link"
FORCE_SCALE = 0.015  # m / N
TORQUE_SCALE = 0.05  # m / Nm
TIMEOUT = 0.5        # この秒数、指令が来なければ消す


class ForceArrow(Node):
    def __init__(self):
        super().__init__("force_arrow")
        self.pub = self.create_publisher(MarkerArray, "force_markers", 10)
        self.create_subscription(Float64MultiArray, "/fake_ft_command/commands", self.on_cmd, 10)
        self.last = self.get_clock().now()
        self.create_timer(0.1, self.on_timer)
        self.shown = False

    def arrow(self, mid, vec, scale, rgb, ns):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns, m.id, m.type, m.action = ns, mid, Marker.ARROW, Marker.ADD
        m.points = [self.pt(0, 0, 0), self.pt(*(v * scale for v in vec))]
        m.scale.x, m.scale.y, m.scale.z = 0.012, 0.025, 0.04  # 軸の太さ、矢じりの幅、矢じりの長さ
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        return m

    @staticmethod
    def pt(x, y, z):
        from geometry_msgs.msg import Point
        return Point(x=float(x), y=float(y), z=float(z))

    def text(self, mid, pos, s, rgb):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns, m.id, m.type, m.action = "label", mid, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position = self.pt(*pos)
        m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
        m.text = s
        return m

    def on_cmd(self, msg):
        d = list(msg.data) + [0.0] * 6
        f, t = d[:3], d[3:6]
        fn = sum(x * x for x in f) ** 0.5
        tn = sum(x * x for x in t) ** 0.5
        self.last = self.get_clock().now()
        arr = MarkerArray()
        red, blue = (1.0, 0.1, 0.1), (0.2, 0.4, 1.0)
        if fn > 1e-6:
            arr.markers.append(self.arrow(0, f, FORCE_SCALE, red, "force"))
            arr.markers.append(self.text(0, [x * FORCE_SCALE * 1.2 for x in f], f"{fn:.0f}N", red))
        if tn > 1e-6:
            arr.markers.append(self.arrow(1, t, TORQUE_SCALE, blue, "torque"))
            arr.markers.append(self.text(1, [x * TORQUE_SCALE * 1.2 for x in t], f"{tn:.0f}Nm", blue))
        if arr.markers:
            self.shown = True
            self.pub.publish(arr)
        else:
            self.clear()

    def on_timer(self):
        if self.shown and (self.get_clock().now() - self.last).nanoseconds * 1e-9 > TIMEOUT:
            self.clear()

    def clear(self):
        m = Marker()
        m.action = Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[m]))
        self.shown = False


def main():
    rclpy.init()
    rclpy.spin(ForceArrow())


if __name__ == "__main__":
    main()
