#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class ProjectedGradientController(Node):
    def __init__(self):
        super().__init__('projected_gradient_ori_controller')
        self.N = 7           # number of joints

        # Parameters
        self.declare_parameter('T', 3.0)  # Trajectory duration
        self.declare_parameter('dt', 0.01)  # Control loop period
        self.declare_parameter('orientation', False)  # Whether to compute orientation Jacobian
        self.T = self.get_parameter('T').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.orientation = self.get_parameter('orientation').get_parameter_value().bool_value

        # Joint limits
        self.LIM_q_max = np.array([ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159])
        self.LIM_q_min = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159])
        self.LIM_dq_max = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
        self.LIM_ddq_max = np.array([10.0] * self.N)  # [rad/s^2]

        # FR3 joint names (7-DOF)
        self.joint_names = [
            'fr3_joint1','fr3_joint2','fr3_joint3',
            'fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7',
        ]

        # Select a singular configuration (example values)
        # self.q_sing = np.array([np.pi/3, 0, -np.pi/2, -np.pi/3, np.pi/2, np.pi/5, -np.pi/5])

        # Simulation values from MATLAB
        self.r_start = np.array([0.36464, -0.055694, 0.79503, 3.1045, 0.5075, 0.1272])
        self.r_end = np.array([0.36464, -0.055694, 0.99503, 1.8823, 0.7, 1.29789])

        if not self.orientation:
            self.r_start = self.r_start[:3]
            self.r_end = self.r_end[:3]

        self.q = np.array([-0.028095, -0.27994, -0.061667, -1.8035, -0.0083122, 1.7527, 0])
        self.dq = np.zeros(self.N)

        # Trajectory delta
        self.delta_r = self.r_end - self.r_start

        # Time
        self.t = 0.0
        self.t_fin = self.T

        self.rows, self.cols = 6 if self.orientation else 3, self.N

        # Initialize Jacobian and pose
        self.J = np.zeros((self.rows, self.N))  # Placeholder for Jacobian
        self.J_dot = np.zeros((self.rows, self.N))  # Placeholder for Jacobian time derivative
        self.pose = np.zeros((self.rows, ))  # Placeholder for end-effector pose
        self.grad_H = np.zeros((self.N, ))  # Placeholder for gradient of manipulability measure


        # Subscribe to Jacobian and position topics
        
        if self.orientation:
            self.create_subscription(
                Float64MultiArray,
                'jacobian_task',
                self.jacobian_callback,
                10)
            self.create_subscription(
                Float64MultiArray,
                'end_effector_pose',
                self.pose_callback,
                10)
        else:
            self.create_subscription(
                Float64MultiArray,
                'jacobian_geom',
                self.jacobian_callback,
                10)
            self.create_subscription(
                Float64MultiArray,
                'end_effector_position',
                self.pose_callback,
                10)
        

        self.create_subscription(
            Float64MultiArray,
            'grad_H',
            self.grad_H_callback,
            10
        )
        self.create_subscription(
            Float64MultiArray,
            'jacobian_dot',
            self.jac_dot_callback,
            10
        )

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # publish initial joint state
        self.publish_joint_state()

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_callback)
        self.get_logger().info('ProjectedGradientController initialized')

    def control_callback(self):
        if self.t > self.t_fin:
            self.get_logger().info('Trajectory complete, shutting down.')
            self.t = 0.0
            self.q = np.array([-0.028095, -0.27994, -0.061667, -1.8035, -0.0083122, 1.7527, 0])
            self.dq = np.zeros(self.N)
            self.publish_joint_state()
            self.get_logger().info('Resetting trajectory.')
            return

        # quintic polynomial and derivative
        tau = self.t / self.T
        s   = 6*tau**5 - 15*tau**4 + 10*tau**3
        ds  = (30*tau**4 - 60*tau**3 + 30*tau**2) / self.T
        dds = (120*tau**3 - 180*tau**2 + 60*tau) / self.T**2

        # Nominal task-space trajectory
        r_nom  = self.r_start + self.delta_r * s
        dr_nom = self.delta_r * ds
        ddr_nom = self.delta_r * dds

        # Projected Gradient step (stub)
        self.ddq = self.proj_grad_step_acc(self.dq, ddr_nom, r_nom, dr_nom)

        # Clamp velocities and positions
        self.ddq = np.clip(self.ddq, -self.LIM_ddq_max, self.LIM_ddq_max)
        self.dq += self.ddq * self.dt
        self.dq = np.clip(self.dq, -self.LIM_dq_max, self.LIM_dq_max)
        self.q += self.dq * self.dt + self.ddq * self.dt**2 / 2.0
        self.q  = np.clip(self.q, self.LIM_q_min, self.LIM_q_max)

        # Publish JointState
        self.publish_joint_state()

        self.t += self.dt


    def proj_grad_step_acc(self, dq, ddr, p_d, dp_d):

        # Current J, p, grad_H from subscriptions
        J = self.J
        J_dot = self.J_dot
        pose = self.pose
        grad_H = self.grad_H

        # Acceleration 
        x_ddot = ddr - J_dot.dot(dq)

        # Pseudoinverse
        pinv_J = np.linalg.pinv(J)

        damp = 2
        q0_ddot = grad_H - damp * dq
        
        # Error
        e = p_d - pose
        e_dot = dp_d - J.dot(dq)
        Kp = 10 * np.eye(self.rows)
        Kd = 5 * np.eye(self.rows)
        PD_control = Kp.dot(e) + Kd.dot(e_dot)

        # Projected gradient update
        ddq = q0_ddot + pinv_J.dot(x_ddot - J.dot(q0_ddot) + PD_control)

        return ddq
    
    def jacobian_callback(self, msg):
        # J = np.array(msg.data).reshape((self.rows, self.N))
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        data = np.array(msg.data)
        self.J = data.reshape((rows, cols))

    def jac_dot_callback(self, msg):
        # J_dot = np.array(msg.data).reshape((self.rows, self.N))
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        data = np.array(msg.data)
        self.J_dot = data.reshape((rows, cols))

    def pose_callback(self, msg):
        self.pose = np.array(msg.data)

    def grad_H_callback(self, msg):
        self.grad_H = np.array(msg.data)


    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q.tolist()
        js.velocity = self.dq.tolist()
        self.get_logger().info(f'Publishing joint state: {js.velocity}')
        self.joint_pub.publish(js)


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