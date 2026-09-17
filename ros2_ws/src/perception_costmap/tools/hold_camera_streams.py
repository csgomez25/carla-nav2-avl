#!/usr/bin/env python3
"""Keep one subscriber on every camera topic the costmap node reads.

The ZED wrapper publishes only while a topic has subscribers, and on this car
it does NOT resume once the count has fallen back to zero: the first
subscription after bring-up streams at 8 Hz, every later one reads nothing and
the zed_node sits at ~8% CPU (measured 2026-09-17). That makes an A/B
impossible -- variant A's node is the one live subscriber, and variant B, which
starts after A exits, gets a dead stream.

Holding these subscriptions open for the whole session keeps the count above
zero, so each costmap node under test joins a stream that is already running.
Subscriptions are raw: the bytes are never deserialised, so this costs almost
nothing and does not compete with the node being measured.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = [
    "/zed_%s/zed_node/rgb/color/rect/image",
    "/zed_%s/zed_node/depth/depth_registered",
    "/zed_%s/zed_node/confidence/confidence_map",
]


class Holder(Node):
    def __init__(self):
        super().__init__("camera_stream_holder")
        self.count = 0
        for cam in ("front", "left", "right"):
            for pattern in TOPICS:
                self.create_subscription(
                    Image, pattern % cam, self._bump,
                    qos_profile_sensor_data, raw=True)
        self.create_timer(10.0, self._report)

    def _bump(self, _msg):
        self.count += 1

    def _report(self):
        self.get_logger().info("holding 9 camera topics, %d msgs" % self.count)


def main():
    rclpy.init()
    node = Holder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
