clc
clear all
digits 4
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")
addpath("./Trajectory/")

% GLOBALS
N = 7; % number of joints
T = 10; % total time for the trajectory [s]


% LIMITS (from Docs)
LIM_q_max       = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]'; % [rad]
LIM_q_min       = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]'; % [rad]
LIM_dq_max      = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
LIM_ddq_max     = 10 * ones(1, N); % [rad/s^2]
LIM_dddq_max    = 5000 * ones(1, N);
LIM_dp_max      = [3.0, 3.0, 3.0, 2.5, 2.5, 2.5]'; % [m/s]x3 [rad/s]x3
LIM_ddp_max     = [9.0, 9.0, 9.0, 17, 17, 17]'; % [m/s^2]x3 [rad/s^2]x3
LIM_dddp_max    = [4500.0, 4500.0, 4500.0, 5000.0, 5000.0, 5000.0]'; % [m/s^3]x3 [rad/s^3]x3


% CHOSEN SINGULARITY: s2 = 0, c3 = 0, c5 = 0 
% (other joints are set to random values)
q_s = {};
p_s = {};
q_s{1} = [pi/3, 0, pi/2, -pi/3, pi/2, pi/5, -pi/5]';
q_s{2} = [pi/3, 0, pi/2, -pi/3, -pi/2, pi/5, -pi/5]';
q_s{3} = [pi/3, 0, -pi/2, -pi/3, pi/2, pi/5, -pi/5]';
q_s{4} = [pi/3, 0, -pi/2, -pi/3, -pi/2, pi/5, -pi/5]';
p_s{1} = get_p(q_s{1});
p_s{2} = get_p(q_s{2});
p_s{3} = get_p(q_s{3});
p_s{4} = get_p(q_s{4});

p_sing = p_s{3};
q_sing = q_s{3};
% fprintf("Before Clamping:\n q_s = [%f, %f, %f, %f, %f, %f, %f]\n", q_sing(1), q_sing(2), q_sing(3), q_sing(4), q_sing(5), q_sing(6), q_sing(7));
% q_sing = clamp_vec(q_sing, LIM_q_min, LIM_q_max); % clamp to limits
% fprintf("After Clamping:\n q_s = [%f, %f, %f, %f, %f, %f, %f]\n", q_sing(1), q_sing(2), q_sing(3), q_sing(4), q_sing(5), q_sing(6), q_sing(7));
% pause;

% DEFINING TRAJECTORY 
% Define dz (di quanto ci dobbiamo muovere in z)
dz = 0.2; % [m]
% singularity at -0.23, 0.28. 0.89
p_start = p_sing;
p_start(3) = p_start(3) - dz/2;

% Setting p_end
p_end = p_sing;
p_end(3) = p_end(3) + dz/2;

% DEFINING ERROR
q_start = num_IK(p_start); % compute inverse kinematics for the start position
% we set an amount of error for the controller to recover
q_start(1) = q_start(1)/2; 
q_start(2) = q_start(2)/2;


t_in = 0; % initial time [s]
t_fin = t_in + T; % final time [s]

% quintic rest-to-rest
syms t_sym real
tau = t_sym/T;

delta_p = p_end - p_start; % change in position
p_d_sym = p_start + delta_p * (6 * tau^5 - 15 * tau^4 + 10 * tau^3); % quintic polynomial
dp_sym = diff(p_d_sym, t_sym);
ddp_sym = diff(dp_sym, t_sym); % first and second derivatives

q_list = []; % to store joint positions
dq_list = []; % to store joint velocities
p_list = []; % to store end-effector positions
error_list = []; % to store error norms

ms_a_list = []; % to store minimum singular values of J_a

qA_idx = [2,4,5]; % indices of joints in A (nonsingular)
qB_idx = [1, 3, 6, 7]; % indices of joints in B (N-M = 4)

dt = 0.01; % time step
t = 0.0;
q = q_start; % initialize joint position
dq = zeros(N, 1); % initialize joint velocity

t_sing = 0;

disp("Simulation values...");
disp(['Start position: p_start = [', num2str(p_start'), ']']);
disp(['Singularity position: p_sing = [', num2str(p_sing'), ']']);
disp(['End position: p_end = [', num2str(p_end'), ']']);
disp(['Start joint angles: q_start = [', num2str(q_start'), ']']);
disp(['Time step: dt = ', num2str(dt), ' s']);
disp(['Total simulation time: t_fin = ', num2str(t_fin), ' s']);
disp("Press enter to start the simulation...");
pause; % wait for user input to start the simulation

while t < t_fin % run for a fixed time

    % Nominal Trajectory
    p_nom = double(subs(p_d_sym, t_sym, t)); % expected end-effector position at time t
    dp_nom = double(subs(dp_sym, t_sym, t)); 
    q_nom = num_IK(p_nom, q); % compute inverse kinematics for the expected end-effector position
    
    % LOGGING errors and pos
        p = get_p(q); % compute current end-effector position
        J = get_J(q);

        J_a = J(:, qA_idx); % (3x7 o 6x7) -> (3xN_a o 6xN_a) where N_a = length(qA_idx)

        ms_a = svds(J_a, 1, 'smallest');
        disp(['Minimum singular value of J_a: ', num2str(ms_a)]);
        ms_a_list = [ms_a_list, ms_a]; % store minimum singular value for plotting later

        dp = J * dq;
        error = p_nom - p;
        norm_e = double(norm(error));
        disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] dp = [', num2str(dp'), ']']);
        disp( ['q = [', num2str(q'), ']']);
        disp(['norm error = ', num2str(norm_e)]);

        error_list = [error_list, norm_e]; % store error norm for plotting later
        q_list = [q_list, q]; % store joint position
        dq_list = [dq_list, dq]; % store joint velocity
        p_list = [p_list, p]; % store end-effector position
    
    % [!] RG step
    dq = reduced_grad_step(q, dp_nom, qA_idx, qB_idx, p_nom); % compute joint velocity using reduced gradient step
    disp(['dq = [', num2str(dq'), ']']);

    % CHECK Limits
    %dq = clamp_vec(dq, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
    disp(['Clamped dq = [', num2str(dq'), ']']);

    q = q + dq * dt; % update joint position
    %q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits

    % if we are near the singularity, we want to save the time in t_sing
    if norm(p-p_sing) < 0.01 % if we are within 0.1 rad of the singularity
            t_sing = t; % save the time
            fprintf("Near singularity at t = %.2f s\n", t_sing);
    end

    t = t + dt; % update time 
end


fin_err = p_end - p;
fprintf("Norm of final error: %.4f\n", norm(fin_err))



% plot joint over time
disp("Simulation finished. Plotting results...");

figure;
% plot svd of J_a over time
plot(ms_a_list, 'b', 'DisplayName', 'Minimum Singular Value of J_a');
xlabel('Time Step');
ylabel('Minimum Singular Value');
title('Minimum Singular Value of J_a Over Time');
grid on;
legend;

figure;

% Plot end-effector position over time
subplot(2, 1, 1);
hold on;
% time = 0:dt:t_fin-dt;
if T > 2
    time = 0:dt:t_fin; % time vector for plotting
else
    time = 0:dt:t_fin-dt; % time vector for plotting
end


plot(time, p_list(1, :), 'b', 'DisplayName', 'Real Position (X)');
plot(time, double(subs(p_d_sym(1), t_sym, time)), 'r--', 'DisplayName', 'Nominal Position (X)');
plot(time, p_list(2, :), 'g', 'DisplayName', 'Real Position (Y)');
plot(time, double(subs(p_d_sym(2), t_sym, time)), 'm--', 'DisplayName', 'Nominal Position (Y)');
plot(time, p_list(3, :), 'c', 'DisplayName', 'Real Position (Z)');
plot(time, double(subs(p_d_sym(3), t_sym, time)), 'k--', 'DisplayName', 'Nominal Position (Z)');
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position Over Time (X, Y, Z)');
legend;
grid on;

% Plot norm of the error over time
subplot(2, 1, 2);
plot(time, error_list, 'g', 'DisplayName', 'Norm of Error');
xlabel('Time (s)');
ylabel('Error Norm (m)');
title('Norm of End-Effector Position Error Over Time');
xline(t_sing, '--r', 'Singularity Crossing', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle', 'HandleVisibility', 'off');
legend;
grid on;


% Plot joint positions
figure;
plot(time, q_list);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Time');
% Add grid and legend
grid on;
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');

% Add a vertical line at t = 0.97
%hold on;
%

% Add dotted lines at q = [-0.092246, 0.3226, -0.067663, -0.11786, -0.077494, 0.56935, 0]
%q_target = [-0.092246, 0.3226, -0.067663, -0.11786, -0.077494, 0.56935, 0];
%for i = 1:length(q_target)
%    yline(q_target(i), '--', ['q', num2str(i), ' = ', num2str(q_target(i))], ...
%        'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle');
%end

% plot joint velocities with bounds for each joint
figure;
for i = 1:N
    subplot(N, 1, i);
    plot(time, dq_list(i, :), 'b', 'DisplayName', ['q', num2str(i), ' Velocity']);
    hold on;
    yline(LIM_dq_max(i), 'r--', 'DisplayName', ['q', num2str(i), ' Max']);
    yline(-LIM_dq_max(i), 'g--', 'DisplayName', ['q', num2str(i), ' Min']);
    xlabel('Time (s)');
    ylabel(['q', num2str(i), ' Velocity (rad/s)']);
    title(['Joint ', num2str(i), ' Velocity Over Time']);
    grid on;
    legend;
end

% plot joint positions with bounds for each joint
figure;
for i = 1:N
    subplot(N, 1, i);
    plot(time, q_list(i, :), 'b', 'DisplayName', ['q', num2str(i), ' Position']);
    hold on;
    yline(LIM_q_max(i), 'r--', 'DisplayName', ['q', num2str(i), ' Max']);
    yline(LIM_q_min(i), 'g--', 'DisplayName', ['q', num2str(i), ' Min']);
    xlabel('Time (s)');
    ylabel(['q', num2str(i), ' Position (rad)']);
    title(['Joint ', num2str(i), ' Position Over Time']);
    grid on;
    legend;
end





% After computing p_start and p_end, add this in the plotting section:
figure
plot3(p_list(1, :), p_list(2, :), p_list(3, :));
hold on;
plot3([p_start(1) p_end(1)], [p_start(2) p_end(2)], [p_start(3) p_end(3)], 'g--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('End-Effector Position Over Time');
scatter3(p_sing(1), p_sing(2), p_sing(3), 50, 'filled', 'MarkerFaceColor', 'k'); % Singularity point in red
scatter3(p_start(1), p_start(2), p_start(3), 10, 'filled', 'MarkerFaceColor', 'g'); % Start point in green
scatter3(p_end(1), p_end(2), p_end(3), 10, 'filled', 'MarkerFaceColor', 'b'); % End point in blue
legend('End-Effector Path', 'Start to End Path', 'Singularity Point', 'Start Point', 'End Point');

dx = 0.5;

xlim([p_start(1) - dx, p_start(1) + dx])
ylim([p_start(2) - dx, p_start(2) + dx])
zlim([p_start(3) - dx, p_start(3) + dx])
grid on;
