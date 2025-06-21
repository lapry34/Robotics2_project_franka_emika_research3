#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class JacobianComputation(Node):
    def __init__(self):
        super().__init__('jacobian_computer_node')

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
        
        # --------------------------------------------------------------------

        # Solvers
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)

        # Joint containers
        num_joints = self.kdl_chain.getNrOfJoints()
        self.joint_positions = PyKDL.JntArray(num_joints)
        self.kdl_jacobian = PyKDL.Jacobian(num_joints)
        self.J_geom = np.zeros((6, num_joints))

        self.rows, self.cols = 6, num_joints
        self.dbg = True  # Flag to log first message only

        # subscribers
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # publishers
        self.jacobian_pub = self.create_publisher(Float64MultiArray, 'jacobian', 10)
        self.rotation_pub = self.create_publisher(Float64MultiArray, 'rotation_matrix', 10)
        self.phi_pub = self.create_publisher(Float64MultiArray, 'euler_angles', 10)

        # Log initialization
        self.get_logger().info('JacobianComputer initialized')

    def joint_state_callback(self, msg: JointState):
        
        self._update_joints(msg)
        self._update_J()

        
        R = self._get_R()
        phi = self._get_phi(R)
        J_task = self._get_task_J(self.J_geom, phi)

        
        if self.dbg:
            self.get_logger().info(f"Received joint states: {msg.position}")
            self.get_logger().info(f"Jacobian computed:\n{J_task}\n\n\n")

            self.dbg = False

        self._publish_matrix(self.jacobian_pub, J_task)
        self._publish_matrix(self.rotation_pub, R)
        self._publish_array(self.phi_pub, phi)




    def _get_R(self) -> np.ndarray:
        """ could also be done by listening to the tf topic: maybe more efficient? """
        # Compute forward kinematics to get the end-effector frame
        end_frame = PyKDL.Frame()
        self.fk_solver.JntToCart(self.joint_positions, end_frame)

        # Extract rotation as a PyKDL Rotation object
        rot = end_frame.M

        # Convert to numpy array
        rot_mat = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                rot_mat[i, j] = rot[i, j]
        return rot_mat


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
        


    def _update_J(self):

        self.jac_solver.JntToJac(self.joint_positions, self.kdl_jacobian)

        for i in range(self.rows):
            for j in range(self.cols):
                self.J_geom[i, j] = self.kdl_jacobian[i, j]


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
    

    def _get_task_J(self, J_geo: np.ndarray, phi:np.ndarray) -> np.ndarray:
        """
        Build task-space Jacobian: J_task = blkdiag(I3, E_inv) * J_geo.
        """
        # Block diagonal 6x6
        E = self._get_E(phi)
        E_inv = np.linalg.inv(E)
        B = np.zeros((6, 6))
        B[:3, :3] = np.eye(3)
        B[3:, 3:] = E_inv
        return B.dot(J_geo)


    def _update_joints(self, msg: JointState):
        for idx, name in enumerate(msg.name):
            if idx < self.joint_positions.rows():
                self.joint_positions[idx] = msg.position[idx]
        # self.get_logger().info(f"Updated joint positions: {self.joint_positions}")



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

