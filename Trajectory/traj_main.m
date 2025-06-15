clc
clear all
digits 4

% addpath("/Matlab_Scripts/Dynamics/") %no dynamics for now...
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")

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

fprintf("Chosen Singularity:\n p_s{3} = [%f, %f, %f]\n", p_s{3}(1), p_s{3}(2), p_s{3}(3));
% pause;

% J_s = get_J(q_sing);
% rank_J = rank(J_s)
%s = svd(J_s);           % s is a vector of singular values sorted in descending order
%min_singular = s(end) % smallest singular value is the last one
% min_singular = svds(J_s, 1, 'smallest'); %computes only the smallest singular value
% disp("Minimum singular value: " + min_singular)

% Define dz (di quanto ci dobbiamo muovere in z)
dz = 0.3; % [m]
% singularity at -0.23, 0.28. 0.89
p_start = p_s{3};
p_start(3) = p_start(3) - dz/2
q_start = num_IK(p_start); % compute inverse kinematics for the start position
q_start(1) = 0.0; % we fix an amount of error for the controller
%check q_start


% Setting p_fin
% p_fin = p_start + dr * T  % <- se dr è costante
p_fin = p_s{3};
p_fin(3) = p_fin(3) + dz/2


J_dot = get_J_dot(q_start, q_start, true); % initial Jacobian time derivative

t_in = 0;
t_fin = 2.0;
T = t_fin - t_in;

% quintic rest-to-rest

p_d = sym([0; 0; 0]);

syms t_sym real
tau = t_sym/T;
for i = 1:3
    dp_i = p_fin(i) - p_start(i);
    % if v_in=v_fin=a_in=a_fin=0, then...
    p_d(i) = p_start(i) + dp_i * (6 * tau^5 - 15 * tau^4 + 10 * tau^3);
end

p_d_sym = p_d;
dr_sym = diff(p_d_sym, t_sym)
%q_old =  [pi/3, pi/3, pi/3, -pi/3, -pi/3, pi/5, -pi/5]';


q_list = []; % to store joint positions
p_list = []; % to store end-effector positions

dt = 0.01; % time step
t = 0.0;
q = q_start; % initialize joint position
dq_old = zeros(N, 1); % initial joint velocity
q_dot = zeros(N, 1); % initialize joint velocity

while t < t_fin % run for a fixed time

    %dq = saturate(dq, Q_dot_max); % saturate joint velocity
    %dq = saturate(dq, Q_ddot_max, dq_old); % saturate joint acceleration

    p = get_p(q); % compute current end-effector position
    J = get_J(q);
    v_ee = J * q_dot;
    dr = double(subs(dr_sym, t_sym, t));
    error = dr - v_ee;
    norm_e = double(norm(error));
    detJJtrans = det(J*J');
    disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] v_ee = [', num2str(v_ee'), ']']);
    disp( ['q = [', num2str(q'), ']']);
    fprintf("det(J*J') = %.4f\n", detJJtrans);
    disp(['norm error = ', num2str(norm_e)]);
    % p = [-0.35603      0.2033      1.0844]

    q_list = [q_list, q]; % store joint position
    p_list = [p_list, p]; % store end-effector position
    
    % TODO: r_dot dovrebbe essere in funzione di t. quinid questa va modificata
    
    p_d = subs(p_d_sym, t_sym, t); % expected end-effector position at time t

    q_dot = proj_grad_step(q, dr, p_d); % compute joint velocity using projected gradient step
    %J_pinv = pinv(J); % compute pseudo-inverse of Jacobian
    %q_dot = J_pinv * dr;
    q = q + q_dot * dt; % update joint position
    t = t + dt; % update time 
end

p_sing = p_s{3};
q_sing = q_s{3};

fin_err = p_fin - p;
fprintf("Norm of final error: %.4f", norm(fin_err))


% Plot joint positions
figure;
plot(0:dt:t_fin-dt, q_list);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Time');
% Add grid and legend
grid on;
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');

% Add a vertical line at t = 0.97
%hold on;
%xline(0.97, '--r', 't = 0.97', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');

% Add dotted lines at q = [-0.092246, 0.3226, -0.067663, -0.11786, -0.077494, 0.56935, 0]
%q_target = [-0.092246, 0.3226, -0.067663, -0.11786, -0.077494, 0.56935, 0];
%for i = 1:length(q_target)
%    yline(q_target(i), '--', ['q', num2str(i), ' = ', num2str(q_target(i))], ...
%        'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');
%end




% After computing p_start and p_fin, add this in the plotting section:
figure
plot3(p_list(1, :), p_list(2, :), p_list(3, :));
hold on;
plot3([p_start(1) p_fin(1)], [p_start(2) p_fin(2)], [p_start(3) p_fin(3)], 'g--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('End-Effector Position Over Time');
scatter3(p_sing(1), p_sing(2), p_sing(3), 50, 'filled', 'MarkerFaceColor', 'k'); % Singularity point in red
%scatter3(0,0,0, 10, 'filled', 'MarkerFaceColor', 'r'); % Singularity point in red
scatter3(p_start(1), p_start(2), p_start(3), 10, 'filled', 'MarkerFaceColor', 'g'); % Start point in green
scatter3(p_fin(1), p_fin(2), p_fin(3), 10, 'filled', 'MarkerFaceColor', 'b'); % End point in blue
legend('End-Effector Path', 'Start to End Path', 'Singularity Point', 'Start Point', 'End Point');

dx = 0.5;

xlim([p_start(1) - dx, p_start(1) + dx])
ylim([p_start(2) - dx, p_start(2) + dx])
zlim([p_start(3) - dx, p_start(3) + dx])
grid on;
