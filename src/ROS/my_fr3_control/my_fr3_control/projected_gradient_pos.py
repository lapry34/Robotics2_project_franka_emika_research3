#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class ProjectedGradientController(Node):
    def __init__(self):
        super().__init__('projected_gradient_controller')
        self.N = 7           # number of joints

        # Parameters
        self.declare_parameter('T', 3.0)  # Trajectory duration
        self.declare_parameter('dt', 0.01)  # Control loop period
        self.T = self.get_parameter('T').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Joint limits
        self.LIM_q_max = np.array([ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159])
        self.LIM_q_min = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159])
        self.LIM_dq_max = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])

        # FR3 joint names (7-DOF)
        self.joint_names = [
            'fr3_joint1','fr3_joint2','fr3_joint3',
            'fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7',
        ]

        # Select a singular configuration (example values)
        # self.q_sing = np.array([np.pi/3, 0, -np.pi/2, -np.pi/3, np.pi/2, np.pi/5, -np.pi/5])

        # Simulation values from MATLAB
        self.p_start = np.array([0.36464, -0.055694, 0.79503])
        self.p_end = np.array([0.36464, -0.055694, 0.99503])
        
        self.q = np.array([-0.028095, -0.27994, -0.061667, -1.8035, -0.0083122, 1.7527, 0])
        self.dq = np.zeros(self.N)

        # Trajectory delta
        self.delta_p = self.p_end - self.p_start

        # Time
        self.t = 0.0
        self.t_fin = self.T

        # Initialize Jacobian and position
        self.J = np.zeros((6, self.N))  # Placeholder for Jacobian
        self.p = np.zeros((3, ))  # Placeholder for end-effector position


        # Subscribe to Jacobian and position topics
        self.create_subscription(
            Float64MultiArray,
            'jacobian',
            self.jacobian_callback,
            10)
        self.create_subscription(
            Float64MultiArray,
            'end_effector_position',
            self.position_callback,
            10)

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # publish initial joint state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q.tolist()
        js.velocity = self.dq.tolist()
        self.joint_pub.publish(js)


        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_callback)
        self.get_logger().info('ProjectedGradientController initialized')

    def control_callback(self):
        if self.t > self.t_fin:
            self.get_logger().info('Trajectory complete, shutting down.')
            # self.timer.cancel()
            # reset and start again
            self.t = 0.0
            self.q = np.array([-0.028095, -0.27994, -0.061667, -1.8035, -0.0083122, 1.7527, 0])
            self.dq = np.zeros(self.N)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = self.q.tolist()
            js.velocity = self.dq.tolist()
            self.joint_pub.publish(js)
            self.get_logger().info('Resetting trajectory.')
            return

        # quintic polynomial and derivative
        tau = self.t / self.T
        s   = 6*tau**5 - 15*tau**4 + 10*tau**3
        ds  = (30*tau**4 - 60*tau**3 + 30*tau**2) / self.T

        # Nominal task-space trajectory
        p_nom  = self.p_start + self.delta_p * s
        dp_nom = self.delta_p * ds

        # Projected Gradient step (stub)
        self.dq = self.proj_grad_step(self.q, dp_nom, p_nom)

        # Clamp velocities and positions
        self.dq = np.clip(self.dq, -self.LIM_dq_max, self.LIM_dq_max)
        self.q  = np.clip(self.q + self.dq * self.dt, self.LIM_q_min, self.LIM_q_max)

        # Publish JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q.tolist()
        js.velocity = self.dq.tolist()
        self.joint_pub.publish(js)

        self.t += self.dt


    def proj_grad_step(self, q, dr, p_d):
        # Determine if orientation is included
        orientation = (dr.size == 6)

        # Current J and p from subscriptions
        J = self.J[:3, :]  # 6x7 or 3x7
        # if not orientation:
        #     J = J[:3, :]
        
        p = self.p

        # Pseudoinverse
        pinv_J = np.linalg.pinv(J)

        # Manipulability measure H = sqrt(det(J J^T))
        def H_man(q_var):
            JJt = J.dot(J.T)
            return np.sqrt(np.linalg.det(JJt))

        # Numerical gradient of H
        grad_H = self.num_diff(H_man, q)

        # Error
        e = p_d - p
        
        Kp = 3 * np.eye(e.size)

        # Projected gradient update
        # dq = grad_H + pinv_J.dot(dr - J.dot(grad_H) + Kp.dot(e))

        # Projected gradient update
        update = dr - J.dot(grad_H) + Kp.dot(e)
        dq_pg  = pinv_J.dot(update)
        dq     = grad_H + dq_pg
        return dq

    def num_diff(self, func, q, eps=1e-6):
        grad = np.zeros_like(q)
        for i in range(q.size):
            dq = np.zeros_like(q)
            dq[i] = eps
            grad[i] = (func(q + dq) - func(q - dq)) / (2 * eps)
        return grad
    
    def jacobian_callback(self, msg):
        # J = np.array(msg.data).reshape((6, self.N))
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        data = np.array(msg.data)
        self.J = data.reshape((rows, cols))

    def position_callback(self, msg):
        self.p = np.array(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectedGradientController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()