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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class JacobianComputer(Node):
    def __init__(self):
        super().__init__('jacobian_computer_node')

        # 1) Declare & fetch the robot_description
        self.declare_parameter('robot_description', '')
        robot_xml = self.get_parameter('robot_description') \
                        .get_parameter_value().string_value
        if not robot_xml:
            self.get_logger().error("robot_description parameter is empty!")
            return

        # 2) Parse into URDF model
        robot = URDF.from_xml_string(robot_xml)

        # 3) Build KDL tree & chain as before
        success, self.kdl_tree = treeFromUrdfModel(robot)
        if not success:
            self.get_logger().error('Failed to parse URDF to KDL tree')
            return
        base_link = 'base'
        end_link  = 'fr3_link8'  # Adjust this to your end-effector link name
        self.kdl_chain    = self.kdl_tree.getChain(base_link, end_link)
        self.jac_solver   = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        num_joints        = self.kdl_chain.getNrOfJoints()
        self.joint_positions = PyKDL.JntArray(num_joints)
        self.jacobian        = PyKDL.Jacobian(num_joints)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # create a publisher for the jacobian
        self.jacobian_pub = self.create_publisher(
            Float64MultiArray,
            'jacobian',
            10
        )
        self.one = True  # Flag to log first message only

        self.get_logger().info('JacobianComputer initialized')

    def joint_state_callback(self, msg: JointState):
        # Map incoming joint positions to KDL joint array
        # Assuming msg.name matches the chain joint ordering
        for idx, name in enumerate(msg.name):
            if idx < self.joint_positions.rows():
                self.joint_positions[idx] = msg.position[idx]

        if self.one:
            self.get_logger().info(f"Received joint states: {msg.position}")
            self.one = False
        # Compute Jacobian
        self.jac_solver.JntToJac(self.joint_positions, self.jacobian)

        jacobian_matrix = [[self.jacobian[i, j] for j in range(self.jacobian.columns())]
                           for i in range(self.jacobian.rows())]
        
        # self.get_logger().info(f"Jacobian computed:\n{jacobian_matrix}\n\n\n")
        
        rows = self.jacobian.rows()
        cols = self.jacobian.columns()

        # build the Float64MultiArray
        array_msg = Float64MultiArray()
        # layout: two dimensions, rows × cols
        array_msg.layout.dim = [
            MultiArrayDimension(label='rows', size=rows, stride=rows*cols),
            MultiArrayDimension(label='cols', size=cols, stride=cols)
        ]
        # flatten row-major
        array_msg.data = [elem for row in jacobian_matrix for elem in row]
        self.jacobian_pub.publish(array_msg)
        

        # J_np = np.array(jacobian_matrix)

        # # Compute manipulability = sqrt(det(J * Jᵀ))
        # JJt = J_np @ J_np.T                 # 6×6 symmetric
        # det_JJt = np.linalg.det(JJt)        # may be tiny or negative (numerical)
        # manipulability = np.sqrt(abs(det_JJt))

        # # Log it
        # self.get_logger().info(f"Manipulability: {manipulability:.6e}")

        # array_msg.data = J_np.ravel().tolist()





def main(args=None):
    rclpy.init(args=args)
    node = JacobianComputer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down JacobianComputer')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
