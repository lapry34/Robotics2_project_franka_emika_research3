%% DH transformation matrices and direct kinematics of a serial robot (SCARA example)
%% 5 Nov 2012 (DH frames assigned as in lecture slides 09_DirectKinematics, A. De Luca)

clear all
clc

%% Define symbolic variables

syms alpha d a theta L L1 L2 L3 L4 a1 a2 a3 a4 d1 d2 d3 d4 d5 d6 d7 H q0 q1 q2 q3 q4 q5 q6 q7 D gamma real
% qn = [0;pi/2;0;pi;pi;0];
% q1 = qn(1);
% q2 = qn(2);
% q3 = qn(3);
% q4 = qn(4);
% q5 = qn(5);
% q6 = qn(6);

%% number and type of joints of SCARA

N=8;
joints_str = 'RRRRRRR-';

assert(N == length(joints_str), "Mismatch between N and length of joints_str");

%% Insert DH table of parameters of SCARA
% DONOT USE N AS SYM
% alpha a d theta
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

q = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]';
% N = 8;
% joints_str = 'RRRRRRR-';

a_i = [0; 0; 0; a4; a5; 0; a7; 0];
d_i = [d1; 0; d3; 0; d5; 0; 0; d_e];
alpha_i = [0; -pi/2; pi/2; pi/2; -pi/2; pi/2; pi/2; 0];
% a_i = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
% d_i = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
theta_i = [q; 0];

DHTABLE = [alpha_i, a_i, d_i, theta_i]

         
%% Build the general Denavit-Hartenberg trasformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrices for each link

A = cell(1,N);
for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

% % Display the rotation matrix R_i
i = 1;
R_01 = A{1}(1:3, 1:3);
R_12 = A{2}(1:3, 1:3);
R_23 = A{3}(1:3, 1:3);
% disp("R_" + i + " = [" + join(string(R_i(1,:)'), " ") + "; " ...
%                    + join(string(R_i(2,:)'), " ") + "; " ...
%                    + join(string(R_i(3,:)'), " ") + "];");

%% Direct kinematics

disp(['N=',num2str(N)])


T = eye(4);

p_vec = [];
% z_vec = [0; 0; 1];
z_vec = [];

p_i = T(1:3, 4);
z_i = T(1:3, 3);
disp("p_0 = [" + join(string(p_i), "; ") + "];");
disp("z_0 = [" + join(string(z_i), "; ") + "];");
p_vec = [p_vec, p_i];
z_vec = [z_vec, z_i];

for i=1:N 
%     disp(i)
%     disp(A{i})
    T = T*A{i};
    T = simplify(T);
    
    % disp p_i and z_i
    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
    disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_vec, p_i];
    z_vec = [z_vec, z_i];
end

disp("__________________________________")


% output TN matrix
% output ON position
% output xN axis
% output yN axis
% output zN axis
T0N = T
p = T(1:3,4)
n=T(1:3,1)
s=T(1:3,2)
a=T(1:3,3)

%% Geometric Jacobian

JP = [];
JO = [];

for i = 1:N
    p_i = p_vec(:, i);
    z_i = z_vec(:, i);
    if joints_str(i) == 'R'
        JP = [JP, cross(z_i, p_vec(:, end) - p_i)];
        JO = [JO, z_i];
    elseif joints_str(i) == 'P'
        JP = [JP, z_i];
        JO = [JO, [0; 0; 0]];
    end
end

J = [JP; JO];
J = simplify(J);
disp("Jacobian matrix:");
disp(J);
size(J)

% jacobian(J'*J, d_e)

J_s = subs(J, [theta3, theta5], [pi/2, 0])
rank(J_s)

J_s = subs(J, [theta2, theta3, theta5], [0, pi/2, pi/2])
rank(J_s)

% disp("Jacobian matrix wrt Frame 1:");
% disp(simplify([R_01.' * JP; R_01.' * JO]));
% disp("Jacobian matrix wrt Frame 2:");
% disp(simplify([R_12.' * R_01.' * JP; R_12.' * R_01.' * JO]));
% disp("Jacobian matrix wrt Frame 3:");
% disp(simplify([R_23.' * R_12.' * R_01.' * JP; R_23.' * R_12.' * R_01.' * JO]));
% J=[R_12.' * R_01.' * JP; R_12.' * R_01.' * JO];
% J = [J(1:2, :); J(6:6, :)];

% disp("Determinant:");
% % disp(simplify(det(JP)));
% % if N == 6
% %     det_j = det(J);
% % else
%     % det_j = det(J * J.');
%     det_j = det(JP);
% % end
% % det_j = det(J(1:2, 1:2));
% disp(simplify(det_j));

% disp(simplify(det(JP*JP.')));
% disp(simplify(det(JO*JO.')));
% disp(simplify(det(JO.'*JO)));

%% change in order to the question
% 1. Find the singularities from det
% 2. Study the singularities changing the code
% ex. 
% det = sin q2 -> q2 = 0, pi in the code
% then compute the null space in order to study the singularities

% 3. if det is difficult, you can use:
% J1 = [simplify(R_i.' * JP); simplify(R_i.' * JO)];
% JP1 = J1(1:3, 1:3);
% JO1 = J1(4:6, :);
% disp(simplify(det(JP1)));
% disp(simplify(det(JO1*JO1.')));


% 4. Null Space and Range Space:
% -
% dim(range(J))   = rank(J)     +     dim(null(J.')) = m - rank(J)     = R^m    
% dim(range(J.')) = rank(J)     +     dim(null(J))   = n - rank(J)     = R^n
% si dice che null(J.') è complementare a range(J) e null(J) è complementare a range(J.')
% range(J)   == direzioni in cui il robot può muoversi
% null(J.')  == direzioni in cui il robot non può muoversi (sing) -> una qualsiasi forza non muove il robot
% null(J)    == configurazioni (joint vel) che non generano movimento
% range(J.') == configurazioni (joint torq) che generano movimento / forze dell`e-e
% Slides 12b.9
%
% -
% null(JP) gives the base B s.t. for each q_dot=alpha*B -> v = JP*q_dot = 0
% null(JO) gives the base B s.t. for each q_dot=alpha*B -> w = JO*q_dot = 0

% -
% range(JP or JO) gives the base B that can generate the velocity v or w
% 
% the range space is given by the columns of JP or JO that are linearly independent
% [Q, ~, E] = qr(JO, 'vector'); % QR decomposition with column pivoting
% r = rank(JO);                 % Determine the rank of J
% range_space = Q(:, 1:r);      % First r columns of Q form the basis for the range space
% disp(simplify(range_space));
% 
% V belongs to the range space of J if V is a linear combination of the columns of J:
% rank(J) = rank([J,V])
% Q: for what q, V belong to the range space of J? A: det([J,V]) = 0

% -
% if q_dot_star = J^-1 * v_desired, then q_dot = q_dot_star + alpha*B -> v = JP*q_dot = v_desired
% so, if the null space is not empty, there are infinite solutions for q_dot that can generate the desired velocity


% 5. Torques and Forces
% tau = J.' * F gives the joint torque to balance the force -Fe=F
% null(J.') gives the base B s.t. for each F=alpha*B -> tau = J.' * F = 0


% 6. Cartesian Acceleration
% p_ddot = J * q_ddot + J_dot * q_dot
% J_dot = [jacobian(J(:,1), [q1, q2]) * [q1_dot;q2_dot], jacobian(J(:,2), [q1, q2])* [q1_dot;q2_dot]];

% J_dot = [
%   [d(J[1,1])/dq1 * q1_dot + d(J[1,1])/dq2 * q2_dot,  d(J[1,2])/dq1 * q1_dot + d(J[1,2])/dq2 * q2_dot;
%   [d(J[2,1])/dq1 * q1_dot + d(J[2,1])/dq2 * q2_dot,  d(J[2,2])/dq1 * q1_dot + d(J[2,2])/dq2 * q2_dot]];

% n(q, q_dot) = J_dot * q_dot = [n1; n2]