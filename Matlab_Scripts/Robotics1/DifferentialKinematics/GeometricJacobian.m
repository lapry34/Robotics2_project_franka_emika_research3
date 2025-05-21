
%% Main function for SCARA direct kinematics
clear all; clc;

syms a1 a2 a3 d1 q1 q2 q3 D real

% Define symbolic variables

% Define DH table and type of joints
N = 3; % Number of joints

joints_str = 'RRR';
joints_var = [q1, q2, q3];

DHTABLE = [
    pi/2 a1 d1 q1;
    0    a2 0  q2;
    pi/2 a3 0  q3;
];

assert(N == length(joints_str), "Mismatch between N and length of joints_str");
assert(N == size(DHTABLE, 1), "Mismatch between N and number of rows in DHTABLE");
assert(N == length(joints_var), "Mismatch between N and length of joints_var");

% Build transformation matrices
A = build_transformation_matrices(DHTABLE);

% Perform direct kinematics
[p_vec, z_vec, T0N] = direct_kinematics(A);

% Extract position and orientation of the end-effector
disp("Position of the end-effector:");
p_ee = T0N(1:3, 4);
disp(p_ee);

% Compute Jacobian
J_geom = compute_geometric_jacobian(p_vec, z_vec, joints_str);
J_analytical = simplify(jacobian(p_ee, joints_var));
disp("Analytical Jacobian matrix:");
disp(J_analytical);

% Analyze singularities
J_L = J_geom(1:3, :);
J_A = J_geom(4:6, :);

detJ = analyze_singularities(J_L, joints_var);

%substitute values q3 = 0 to determinant
detJ = subs(detJ, q3, 0);
disp("Determinant of the Jacobian with q3 = 0:");
disp(detJ);

p_3d = [0; 0; D; 1];

p_0d =  T0N * p_3d;
p_0d = simplify(p_0d(1:3));
disp("Desired position of the end-effector:");
disp(p_0d);

q = [0 pi/2 0];
q_dot = [0, pi/4, pi/2];
J_comp = subs(jacobian(p_0d, joints_var), joints_var, q);
p_dot = J_comp * q_dot';

p_dot = subs(p_dot, [a1, a2, a3, d1, D], [0.04, 0.445, 0.04, 0.33, 0.52]);
p_dot = double(p_dot);
disp("P_0d velocity:");
disp(p_dot);

%extract rotation matrix from T0N
R0N = T0N(1:3, 1:3);
p_0 = R0N * [0; 0; D];
J_comp = transformation_geometric_jacobian(J_geom, eye(3), p_0); %the first moves the 0, the second moves the end effector
J_comp = simplify(J_comp);
J_comp = J_comp(1:3, :);

J_comp = subs(J_comp, joints_var, q);
p_dot = J_comp * q_dot';
p_dot = subs(p_dot, [a1, a2, a3, d1, D], [0.04, 0.445, 0.04, 0.33, 0.52]);
p_dot = double(p_dot);
disp("P_0d velocity with Geometric Jacobian:");
disp(p_dot);


function A = build_transformation_matrices(DHTABLE)
    %% Build transformation matrices for each link
    N = size(DHTABLE, 1);
    A = cell(1, N);

    for i = 1:N
        alpha = DHTABLE(i, 1);
        a = DHTABLE(i, 2);
        d = DHTABLE(i, 3);
        theta = DHTABLE(i, 4);

        TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                  0             sin(alpha)             cos(alpha)            d;
                  0               0                      0                   1];

        A{i} = TDH;
    end
end

function [p_vec, z_vec, T0N] = direct_kinematics(A)
    %% Perform direct kinematics
    T = eye(4);
    N = length(A);


    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    disp("p_0 = [" + join(string(p_i), "; ") + "];");
    disp("z_0 = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_i];
    z_vec = [z_i];

    for i = 1:N
        T = T * A{i};
        T = simplify(T);
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);
        disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
        disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end

    T0N = T;
    fprintf("\n\n__________________________________\n\n");
    disp("Final Transformation Matrix T0N:");
    disp(T0N);
end

function J = compute_geometric_jacobian(p_vec, z_vec, joints_str)
    %% Compute the geometric Jacobian
    JP = [];
    JO = [];

    N = length(joints_str);

    for i = 1:N
        p_i = p_vec(:, i);
        z_i = z_vec(:, i);
        if joints_str(i) == 'R'
            JP = [JP, cross(z_i, p_vec(:, end) - p_i)];
            JO = [JO, z_i];
        else
            JP = [JP, z_i];
            JO = [JO, [0; 0; 0]];
        end
    end

    J = [JP; JO];
    J = simplify(J);
    disp("Geometric Jacobian matrix:");
    disp(J);
end

function J = transformation_geometric_jacobian(J, R, r)
    %% Compute the geometric Jacobian for a transformation matrix
    M1 = [R, zeros(3); zeros(3), R];
    M2 = [eye(3), -skew(r); zeros(3), eye(3)];
    J = M1 * M2 * J;
end

function det_J = analyze_singularities(J, joints_var)
    %% Analyze singularities based on Jacobian determinant

    %check if the jacobian is square
    [rows, cols] = size(J);

    JJ = 0;

    if rows == cols
        JJ = simplify(J);
    else 
        if rows < cols
            JJ = simplify(J * J');
            fprintf("Jacobian matrix is not square (%d x %d), using J * J'\n", rows, cols);
        else
            JJ = simplify(J' * J);
            fprintf("Jacobian matrix is not square (%d x %d), using J' * J\n", rows, cols);
        end
    end

    det_J = simplify(det(JJ));

    disp("Determinant of the Jacobian:");
    disp(det_J);

    singular_points = solve(det_J == 0, joints_var);


    if isempty(singular_points)
        disp("No singular configurations found.");
    else 
        disp("Singular configurations: ");
        disp(singular_points);
    end

end

function inverseT = invT(T)
    %% Compute the inverse of a transformation matrix
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    inverseT = [R' -R'*p; 0 0 0 1];
end

function S = skew(v)
    %% Compute the skew-symmetric matrix of a vector
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
