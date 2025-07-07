#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
import time

class SimpleOffboard(Node):
    def __init__(self):
        super().__init__('simple_offboard')
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 'init'
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.last_request = self.start_time
        self.forward_start = None
        self.get_logger().info('Waiting for arming and set_mode services...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        self.get_logger().info('Ready!')

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        # Publish setpoints at 10Hz
        self.local_pos_pub.publish(self.pose)
        if self.state == 'init' and now - self.start_time > 2:
            self.arm()
            self.state = 'arming'
        elif self.state == 'arming' and now - self.last_request > 2:
            self.set_mode('OFFBOARD')
            self.state = 'offboard'
        elif self.state == 'offboard' and now - self.start_time > 8:
            self.pose.pose.position.z = 3.0
            self.get_logger().info('Takeoff to 3m')
            if now - self.start_time > 15:
                self.pose.pose.position.x = 10.0
                self.get_logger().info('Moving forward 10m')
                if self.forward_start is None:
                    self.forward_start = now
                if now - self.forward_start > 10:
                    self.land()
                    self.state = 'landed'

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.last_request = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info('Arming...')

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.last_request = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f'Setting mode: {mode}')

    def land(self):
        self.pose.pose.position.z = 0.0
        self.get_logger().info('Landing...')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 