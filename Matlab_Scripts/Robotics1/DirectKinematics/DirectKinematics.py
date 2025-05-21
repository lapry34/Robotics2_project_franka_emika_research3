from sympy import symbols, cos, sin, pi, atan2, sqrt, Matrix, simplify

# Define symbols
#d1, theta2 = symbols('d_1 θ_2')
d1, theta2 = symbols('d_1 theta_2')

# Define DH parameters for two joints
# Note: Each row is [alpha, a, d, theta] in the specified order
# parameters are used for trasformations in inverse order as in the book (theta, d, a, alpha)
DH = [
    [pi/2, 0, d1, 0.5],   # First joint (prismatic)
    [0, 0.3, 0, theta2]   # Second joint (revolute)
]

def print_matlab(mat):
    print("[")
    for row in mat.tolist():

        for i, elem in enumerate(row):
            if i == len(row) - 1:
                print(elem, end=";")
            else:
                print(elem, end=",")
        print()
    print("]")

# Direct kinematics function
def direct_kinematics(DH_params):
    T = Matrix.eye(4)  # Initialize transformation matrix as identity
    for alpha, a, d, theta in DH_params:
        # Create transformation matrix for the current joint
        Ti = Matrix([
            [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
            [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])
        T *= Ti  # Update the total transformation matrix
        T = simplify(T)
    return T

# Compute the transformation matrix
T = direct_kinematics(DH)

# Display the transformation matrix
print("Transformation matrix T from base to end-effector:")
print_matlab(T)

# Extract the rotation matrix from T
R = T[:3, :3]
print("\nRotation matrix:")
print_matlab(R)

# Extract the end-effector position from T
position = T[:3, 3]
print("\nEnd-effector position:")
print_matlab(position)

# Compute the orientation angles around x, y, and z axes
# Rename them as b1, b2, b3
b1 = atan2(R[2, 1], R[2, 2])  # Rotation around x-axis
b2 = atan2(-R[2, 0], sqrt(R[2, 1]**2 + R[2, 2]**2))  # Rotation around y-axis
b3 = atan2(R[1, 0], R[0, 0])  # Rotation around z-axis

b1 = simplify(b1)
b2 = simplify(b2)
b3 = simplify(b3)

# Display the angles
print("\nOrientation angle (b1) around x-axis:")
print(b1)
print("Orientation angle (b2) around y-axis:")
print(b2)
print("Orientation angle (b3) around z-axis:")
print(b3)
