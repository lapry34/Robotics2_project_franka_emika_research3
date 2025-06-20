#!/usr/bin/env python3
"""
ROS2 node that publishes one random joint velocity and updates
all joint positions by integrating that velocity at 1 Hz.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import math

class RandomJointPublisherFR3(Node):
    def __init__(self):
        super().__init__('random_joint_publisher_fr3')
        self.dt = 0.001
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(self.dt, self.publish_and_integrate)

        # FR3 joint names (7-DOF)
        self.joint_names = [
            'fr3_joint1','fr3_joint2','fr3_joint3',
            'fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7',
        ]
        # Use realistic limits if you have them; here ±π rad
        self.joint_limits = [(-math.pi/2, math.pi/2) for _ in self.joint_names]
        # Start all joints at zero (or any neutral pose)
        # random initial position
        self.current_positions = [
            random.uniform(low, high) for low, high in self.joint_limits
        ]
        self.current_positions[0] = -math.pi
        self.vel = 1.0  # rad/s

        self.get_logger().info('RandomJointVelocityPublisherFR3 initialized')

    def publish_and_integrate(self):
        # 1) pick a random joint and velocity
        idx = 0

        # 2) integrate velocity over dt = 1 second
        new_pos = self.current_positions[idx] + self.vel * self.dt
        # 3) clamp within limits
        low, high = self.joint_limits[idx]
        self.current_positions[idx] = max(low, min(high, new_pos))

        # if min or max invert velocity
        if self.current_positions[idx] == low or self.current_positions[idx] == high:
            self.vel = -self.vel


        # 4) prepare full JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self.joint_names
        msg.position = self.current_positions
        # velocities: zero except our moving joint
        msg.velocity = [self.vel if i == idx else 0.0
                        for i in range(len(self.joint_names))]
        msg.effort   = []

        self.pub.publish(msg)
        # self.get_logger().info(
        #     f'Moved {self.joint_names[idx]} → pos {self.current_positions[idx]:.3f} '
        #     f'(vel {self.vel:.3f} rad/s)'
        # )

def main(args=None):
    rclpy.init(args=args)
    node = RandomJointPublisherFR3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
