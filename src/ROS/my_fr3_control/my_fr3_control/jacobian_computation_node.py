#!/usr/bin/env python3
"""
ROS2 Python node to compute and print the manipulator Jacobian for the Franka Research 3 (FR3) arm.

This node subscribes to JointState messages, computes the geometric Jacobian for the current set of joint angles,
and prints it to the console.

Dependencies:
  - rclpy
  - sensor_msgs
  - geometry_msgs
  - tf_transformations or sympy / custom

Usage:
  1. Make sure your FR3 URDF is loaded and a JointState publisher (e.g., the random joint publisher) is running.
  2. Run this node:
       ros2 run my_fr3_control jacobian_computation_node

Note:
  Requires `urdf_parser_py` and `kdl_parser_py` for parsing the URDF into a KDL chain,
  and `PyKDL` for Jacobian computation.
"""
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class JacobianComputation(Node):
    def __init__(self):
        super().__init__('jacobian_computer_node')

        # Parameters
        self.declare_parameter('orientation', False)  # Whether to compute orientation Jacobian
        self.orientation = self.get_parameter('orientation').get_parameter_value().bool_value 
        self.declare_parameter('acceleration', False)  # Whether to compute PG on acceleration 
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().bool_value 
        

        # Declare & fetch the robot_description
        self.declare_parameter('robot_description', '')
        robot_xml = self.get_parameter('robot_description') \
                        .get_parameter_value().string_value
        if not robot_xml:
            self.get_logger().error("robot_description parameter is empty!")
            return

        # Parse into URDF model &  Build KDL tree & chain
        robot = URDF.from_xml_string(robot_xml)

        success, self.kdl_tree = treeFromUrdfModel(robot)
        if not success:
            self.get_logger().error('Failed to parse URDF to KDL tree')
            return
        # Chain
        base_link = 'base'
        end_link = 'fr3_link8'
        self.kdl_chain = self.kdl_tree.getChain(base_link, end_link)
        self.num_joints = self.kdl_chain.getNrOfJoints()
        
        # --------------------------------------------------------------------
        self.rows, self.cols = 6 if self.orientation else 3, self.num_joints
        self.dbg = True  # Flag to log first message only

        # Solvers
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)

        # Joint containers
        self.joint_positions = PyKDL.JntArray(self.num_joints)
        self.joint_velocities = PyKDL.JntArray(self.num_joints)
        self.kdl_jacobian = PyKDL.Jacobian(self.num_joints)
        self.J_geom = np.zeros((self.rows, self.num_joints))
        self.J_dot = np.zeros((self.rows, self.num_joints))


        # subscribers
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # publishers
        # self.jacobian_pub = self.create_publisher(Float64MultiArray, 'jacobian', 10)
        self.grad_H_pub = self.create_publisher(Float64MultiArray, 'grad_H', 10)
        
        if self.orientation:
            self.pose_ori_pub = self.create_publisher(Float64MultiArray, 'end_effector_pose', 10)
            self.jac_task_pub = self.create_publisher(Float64MultiArray, 'jacobian_task', 10)
            # self.rotation_pub = self.create_publisher(Float64MultiArray, 'rotation_matrix', 10)
            # self.phi_pub = self.create_publisher(Float64MultiArray, 'euler_angles', 10)
        else:
            self.position_pub = self.create_publisher(Float64MultiArray, 'end_effector_position', 10)
            self.jac_geom_pub = self.create_publisher(Float64MultiArray, 'jacobian_geom', 10)

        if self.acceleration:
            self.jac_dot_pub = self.create_publisher(Float64MultiArray, 'jacobian_dot', 10)


        # .....................................................................................
        # Publishers for RViz markers
        self.traj_marker_pub = self.create_publisher(Marker, 'ee_trajectory', 10)

        # Keep a history of end‐effector points
        self.ee_history: List[Point] = []

        # Pre‐configure the trajectory marker as POINTS instead of LINE_STRIP
        self.traj_marker = Marker()
        self.traj_marker.header.frame_id = base_link
        self.traj_marker.ns = "ee_trajectory"
        self.traj_marker.id = 1
        self.traj_marker.type = Marker.POINTS
        self.traj_marker.action = Marker.ADD

        # For POINTS, you set both x and y scale to control point size:
        self.traj_marker.scale.x = 0.01  # point width (meters)
        self.traj_marker.scale.y = 0.01  # point height (meters)

        self.traj_marker.color.r = 0.0
        self.traj_marker.color.g = 1.0
        self.traj_marker.color.b = 0.0
        self.traj_marker.color.a = 0.5

        # Log initialization
        self.get_logger().info('JacobianComputer initialized')

    def joint_state_callback(self, msg: JointState):
        
        self._update_joints(msg)
        self._update_J_geom(self.J_geom, self.joint_positions)

        grad_H = self._get_grad_H(self.H_man, self.joint_positions, self.joint_velocities, self.J_dot)
        self._publish_array(self.grad_H_pub, grad_H)

        # Compute forward kinematics to get the end-effector frame
        end_frame = PyKDL.Frame()
        self.fk_solver.JntToCart(self.joint_positions, end_frame)

        p = np.array([end_frame.p[i] for i in range(3)])  # TODO: can be sped up


        if self.orientation:
            R = np.array([[end_frame.M[i, j] for j in range(3)] for i in range(3)])  # TODO: can be sped up

            phi = self._get_phi(R)
            # self.get_logger().info(f"Computed Euler angles: {phi* 180/np.pi} degrees")
            J_task = self._get_task_J(self.J_geom, phi)

            pose = np.concatenate((p, phi))
            self._publish_array(self.pose_ori_pub, pose)
            self._publish_matrix(self.jac_task_pub, J_task)

        else:
            self._publish_array(self.position_pub, p)
            self._publish_matrix(self.jac_geom_pub, self.J_geom)

        if self.acceleration:
            self._publish_matrix(self.jac_dot_pub, self.J_dot)


        ee_pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        self.ee_history.append(ee_pt)
        if len(self.ee_history) > 5000:
            self.ee_history.pop(0)

        self.traj_marker.header.stamp = self.get_clock().now().to_msg()
        # assign the list of points directly:
        self.traj_marker.points = list(self.ee_history)
        self.traj_marker_pub.publish(self.traj_marker)

        
        if self.dbg:
            # self.get_logger().info(f"Received joint states: {msg.position}")
            # if self.orientation:
            #     self.get_logger().info(f"Jacobian computed:\n{J_task}\n\n\n")
            # else:
            #     self.get_logger().info(f"Jacobian computed:\n{self.J_geom}\n\n\n")
        
            # E = self._get_E(phi) 
            # E_inv = np.linalg.inv(E)
            # self.get_logger().info(f"Computed E:\n{E}\n")
            # self.get_logger().info(f"Computed E_inv:\n{E_inv}\n\n\n")
            # self.get_logger().info("-"*30)

            self.dbg = False

        


    def _update_joints(self, msg: JointState):
        if not msg.velocity:
            self.get_logger().info("JointState message does not contain velocity information. Defaulting to zero velocities.")
        for idx, name in enumerate(msg.name):
            if idx < self.joint_positions.rows():
                self.joint_positions[idx] = msg.position[idx]
            if msg.velocity and idx < self.joint_velocities.rows():
                self.joint_velocities[idx] = msg.velocity[idx]
            else:
                self.joint_velocities[idx] = 0.0  # Default to zero if no velocity provided
                # self.get_logger().info(f"Joint velocity for {name} not provided or index out of range.")

    def _update_J_geom(self, J_geom_out, q_in: PyKDL.JntArray):
        
        self.jac_solver.JntToJac(q_in, self.kdl_jacobian)

        for i in range(self.rows):
            for j in range(self.cols):
                J_geom_out[i, j] = self.kdl_jacobian[i, j]


    def H_man(self, J: np.ndarray) -> float:
        return np.sqrt(np.linalg.det(J.dot(J.T)))

    
    def _get_grad_H(self, func, q: PyKDL.JntArray, dq: PyKDL.JntArray, J_dot: np.ndarray, eps = 1e-6) -> np.ndarray:
        """
        Compute the gradient of the manipulator function H with respect to joint angles q.
        Updates J_dot if acceleration is True.
        """
        
        def get_J(q_var: PyKDL.JntArray) -> np.ndarray:
            J = np.zeros((self.rows, self.cols))
            self._update_J_geom(J, q_var)

            if self.orientation:
                J = self._get_task_J(J, q_var=q_var)
            return J
        
        if self.acceleration:
            J_dot = np.zeros((self.rows, self.cols))
        
        grad = np.zeros(self.num_joints)
        q_cpy = PyKDL.JntArray(self.num_joints)
        PyKDL.SetToZero(q_cpy)
        PyKDL.Add(q, q_cpy, q_cpy)

        for i in range(self.num_joints):

            q_cpy[i] = q[i] + eps
            J_plus = get_J(q_cpy)

            q_cpy[i] = q[i] - eps
            J_minus = get_J(q_cpy)

            if self.acceleration:
                dJ_dqk = (J_plus - J_minus) / (2.0 * eps)
                J_dot += dJ_dqk * dq[i]        

            grad[i] = (func(J_plus) - func(J_minus)) / (2.0 * eps)
        
        return grad


    def _get_phi(self, R: np.ndarray) -> np.ndarray:

        # phi2 singular at ±90° where cos(phi2)=0
        phi2 = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
        cos_phi2 = np.cos(phi2)
        # avoid division by zero
        if np.isclose(cos_phi2, 0.0):
            self.get_logger().warn('Singularity detected in Euler conversion')
        phi3 = np.arctan2(-R[0, 1]/cos_phi2, R[0, 0]/cos_phi2)
        phi1 = np.arctan2(-R[1, 2]/cos_phi2, R[2, 2]/cos_phi2)
        return np.array([phi1, phi2, phi3])


    def _get_E(self, phi: np.ndarray) -> np.ndarray:
        """
        Compute the differential angle mapping E(phi) s.t. omega = E * phi_dot.
        phi: [phi1, phi2, phi3].
        """
        phi1, phi2 = phi[0], phi[1]
        E = np.array([
            [1,       0,             np.sin(phi2)],
            [0, np.cos(phi1), -np.cos(phi2)*np.sin(phi1)],
            [0, np.sin(phi1),  np.cos(phi1)*np.cos(phi2)]
        ])
        return E
    

    def _get_task_J(self, J_geo: np.ndarray, phi:np.ndarray = None, q_var: PyKDL.JntArray = None) -> np.ndarray:
        """
        Build task-space Jacobian: J_task = blkdiag(I3, E_inv) * J_geo.
        """
        if phi is None:
            if q_var is not None:
                end_frame = PyKDL.Frame()
                self.fk_solver.JntToCart(q_var, end_frame)

                R = np.array([[end_frame.M[i, j] for j in range(3)] for i in range(3)])
                phi = self._get_phi(R)
            else:
                raise ValueError("Either phi or q_var must be provided to compute the task Jacobian.")
        
        # Block diagonal 6x6
        E = self._get_E(phi)
        E_inv = np.linalg.inv(E)
        B = np.zeros((6, 6))
        B[:3, :3] = np.eye(3)
        B[3:, 3:] = E_inv
        return B.dot(J_geo)



    def _publish_matrix(self, pub, data: np.ndarray):
        msg = Float64MultiArray()
        rows, cols = data.shape
        msg.layout.dim = [
            MultiArrayDimension(label='rows', size=rows, stride=rows * cols),
            MultiArrayDimension(label='cols', size=cols, stride=cols)
        ]
        msg.data = data.ravel().tolist()
        pub.publish(msg)

    def _publish_array(self, pub, data: np.ndarray):
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label='size', size=data.size, stride=1)]
        msg.data = data.flatten().tolist()
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JacobianComputation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down JacobianComputer')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

