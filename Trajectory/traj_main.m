clc
clear
digits 4

% addpath("/Matlab_Scripts/Dynamics/") %no dynamics for now...
addpath("../Matlab_Scripts/Redundancy/")
addpath("../Matlab_Scripts/Robotics1/")

% From https://frankaemika.github.io/docs
syms q1 q2 q3 q4 q5 q6 q7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

% a4 =0.0825; 
% a5 =-0.0825;
% a7 = 0.088;
% d3 = 0.316; d5 = 0.384; d_e = 0.107;
q = [q1, q2, q3, q4, q5, q6, q7]';

a_i = [0; 0; 0; a4; a5; 0; a7; 0];
d_i = [d1; 0; d3; 0; d5; 0; 0; d_e]; 
alpha_i = [0; -pi/2; pi/2; pi/2; -pi/2; pi/2; pi/2; 0];

% a_i = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
% d_i = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
q_i = [q; 0];

N = 7;
Q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]'; % [rad]
Q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]'; % [rad]
Q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
Q_ddot_max = 10 * ones(1, N); % [rad/s^2]
Q_dddot_max = 5000 * ones(1, N);
P_dot_max = [3.0, 3.0, 3.0, 2.5, 2.5, 2.5]'; % [m/s]x3 [rad/s]x3
P_ddot_max = [9.0, 9.0, 9.0, 17, 17, 17]'; % [m/s^2]x3 [rad/s^2]x3
P_dddot_max = [4500.0, 4500.0, 4500.0, 5000.0, 5000.0, 5000.0]'; % [m/s^3]x3 [rad/s^3]x3



% T = get_T([0,0,0,0,0,0,0]');
% p = get_p([0,0,0,0,0,0,0]');

% the singularity we are interest in is at s2 = 0, c3 = 0, c5 = 0 
q_s = {};
p_s = {};
q_s{1} = [pi/3, 0, pi/2, -pi/3, pi/2, pi/5, -pi/5]';
p_s{1} = get_p(q_s{1});
q_s{2} = [pi/3, 0, pi/2, -pi/3, -pi/2, pi/5, -pi/5]';
p_s{2} = get_p(q_s{2});
q_s{3} = [pi/3, 0, -pi/2, -pi/3, pi/2, pi/5, -pi/5]';
p_s{3} = get_p(q_s{3});
q_s{4} = [pi/3, 0, -pi/2, -pi/3, -pi/2, pi/5, -pi/5]';
p_s{4} = get_p(q_s{4});

fprintf("Chosen Singularity:\n p_s{2} = [%f, %f, %f]\n", p_s{2}(1), p_s{2}(2), p_s{2}(3));
% pause;

% J_s = get_J(q_sing);
% rank_J = rank(J_s)
%s = svd(J_s);           % s is a vector of singular values sorted in descending order
%min_singular = s(end) % smallest singular value is the last one
% min_singular = svds(J_s, 1, 'smallest'); %computes only the smallest singular value
% disp("Minimum singular value: " + min_singular)

q_start = num_IK([0, 0.287938, 0.895027]')
pause;

dt = 0.01; % time step
dr = [p_s{2}; 0; 0; 0];
dq_old = zeros(N, 1); % initial joint velocity
q_old =  [pi/3, pi/3, pi/3, -pi/3, -pi/3, pi/5, -pi/5]';

% q = q_old + dt * dq_old
% J = get_J(q)
% pinv_J = pinv(J)

t = 0.0;
while t < 10.0

    q = q_old + dt * dq_old;
    t = t + dt;
    
    dq = proj_grad_step(q, dr);
    
    dq = saturate(dq, Q_dot_max); % saturate joint velocity
    dq = saturate(dq, Q_ddot_max, dq_old); % saturate joint acceleration

    q_old = q; % update old joint position
    dq_old = dq; % update old joint velocity

end