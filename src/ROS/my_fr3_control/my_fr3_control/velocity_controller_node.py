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
        self.declare_parameter('T', 1.5)  # Trajectory duration
        self.declare_parameter('dt', 0.001)  # Control loop period
        self.declare_parameter('orientation', False)  # Whether to compute orientation Jacobian
        self.declare_parameter('is_RG', False)  # Whether to compute reduced gradient ro projected gradient
        self.T = self.get_parameter('T').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.orientation = self.get_parameter('orientation').get_parameter_value().bool_value
        self.is_RG = self.get_parameter('is_RG').get_parameter_value().bool_value

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
        self.r_start = np.array([0.36464, -0.055694, 0.79503, 3.1045, 0.5075, 0.1272])
        self.r_end = np.array([0.36464, -0.055694, 0.99503, 1.8823, 0.7, 1.29789])

        if not self.orientation:
            self.r_start = self.r_start[:3]
            self.r_end = self.r_end[:3]

        self.q = np.array([-0.028095, -0.27994, -0.061667, -1.8035, -0.0083122, 1.7527, 0]) # configuration with error in the end-effector position
        # self.q = np.array([-0.0562, -0.5599, -0.061667, -1.8035, -0.0083122, 1.7527, 0]) #nominal configuration

        self.dq = np.zeros(self.N)

        # Trajectory delta
        self.delta_r = self.r_end - self.r_start

        # Time
        self.t = 0.0
        self.t_fin = self.T

        self.rows, self.cols = 6 if self.orientation else 3, self.N

        # Initialize Jacobian and pose
        self.J = np.zeros((self.rows, self.N))  # Placeholder for Jacobian
        self.pose = np.zeros((self.rows, ))  # Placeholder for end-effector pose
        self.grad_H = np.zeros((self.N, ))  # Placeholder for gradient of manipulability measure

        # J_a and J_b column indices for reduced gradient
        if self.orientation:
            self.qA_idx = np.array([0,1,2,3,4,5])
            self.qB_idx = np.array([6])
        else:
            self.qA_idx = np.array([1,3,4])
            self.qB_idx = np.array([0,2,5,6])


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
            
            # J_a and J_b column indices for reduced gradient
            if self.orientation:
                self.qA_idx = np.array([0,1,2,3,4,5])
                self.qB_idx = np.array([6])
            else:
                self.qA_idx = np.array([1,3,4])
                self.qB_idx = np.array([0,2,5,6])

            self.publish_joint_state()
            self.get_logger().info('Resetting trajectory.')
            return
        
        if self.is_RG and self.t > self.T / 2:
            if self.orientation:
                self.qA_idx = np.array([1,2,3,4,5,6])
                self.qB_idx = np.array([0])
            else:
                self.qA_idx = np.array([0,3,4])
                self.qB_idx = np.array([1,2,5,6])

        # quintic polynomial and derivative
        tau = self.t / self.T
        s   = 6*tau**5 - 15*tau**4 + 10*tau**3
        ds  = (30*tau**4 - 60*tau**3 + 30*tau**2) / self.T

        # Nominal task-space trajectory
        r_nom  = self.r_start + self.delta_r * s
        dr_nom = self.delta_r * ds

        if self.is_RG:
            # Reduced Gradient step
            self.dq = self.reduced_grad_step(dr_nom, r_nom)
        else:
            # Projected Gradient step
            self.dq = self.proj_grad_step(dr_nom, r_nom)

        # Clamp velocities and positions
        self.dq = np.clip(self.dq, -self.LIM_dq_max, self.LIM_dq_max)
        self.q  = np.clip(self.q + self.dq * self.dt, self.LIM_q_min, self.LIM_q_max)

        # Publish JointState
        self.publish_joint_state()

        self.t += self.dt


    def proj_grad_step(self, dr, r_d):

        # Current J, p, grad_H from subscriptions
        J = self.J
        pose = self.pose
        grad_H = self.grad_H

        # Pseudoinverse
        pinv_J = np.linalg.pinv(J)

        # Error
        e = r_d - pose

        Kp = 3 * np.eye(self.rows)

        # Projected gradient update
        dq = grad_H + pinv_J.dot(dr - J.dot(grad_H) + Kp.dot(e))

        return dq
    
    def reduced_grad_step(self, dr, r_d):
        # Current J, p, grad_H from subscriptions
        J = self.J
        pose = self.pose
        grad_H = self.grad_H

        # M = self.rows : 3 or 6  -> N-M = 4 or 1
        Id = np.eye(self.N - self.rows)
        J_a = J[:, self.qA_idx] # (3x3)
        J_b = J[:, self.qB_idx] # (3x4)

        # Pseudoinverse
        pinv_J_a = np.linalg.pinv(J_a)
        # pinv_J_a = np.linalg.inv(J_a)

        F = np.hstack([-(pinv_J_a @ J_b).T, Id])  # (N-M x N) matrix for reduced gradient step
        grad_H_b_prime = (F @ grad_H)  # (N-M x 1) gradient of H with respect to qB modified for RG.

        e = r_d - pose  # error vector
        Kp = 3 * np.eye(self.rows)  # proportional gain matrix

        dq_b = grad_H_b_prime  # joint velocities for B
        dq_a = pinv_J_a @ (dr - J_b @ dq_b + Kp @ e)  # joint velocities for A (N_a x 1)

        dq = np.zeros(self.N)
        dq[self.qA_idx] = dq_a  # assign joint velocities for A
        dq[self.qB_idx] = dq_b  # assign joint velocities for B

        return dq
    
    def jacobian_callback(self, msg):
        # J = np.array(msg.data).reshape((6, self.N))
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        data = np.array(msg.data)
        self.J = data.reshape((rows, cols))

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