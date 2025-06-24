clc
clear
close all
digits 4
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")
addpath("./Trajectory/")
addpath("./Plots/")

%% GLOBALS
N = 7; % number of joints
T = 7; % total time for the trajectory [s]

%% LIMITS (from Docs)
LIM_q_max       = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]'; % [rad]
LIM_q_min       = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]'; % [rad]
LIM_dq_max      = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
LIM_ddq_max     = 10 * ones(1, N); % [rad/s^2]
LIM_dddq_max    = 5000 * ones(1, N);
LIM_dp_max      = [3.0, 3.0, 3.0, 2.5, 2.5, 2.5]'; % [m/s]x3 [rad/s]x3
LIM_ddp_max     = [9.0, 9.0, 9.0, 17, 17, 17]'; % [m/s^2]x3 [rad/s^2]x3
LIM_dddp_max    = [4500.0, 4500.0, 4500.0, 5000.0, 5000.0, 5000.0]'; % [m/s^3]x3 [rad/s^3]x3

%% CHOSEN SINGULARITY: s2 = 0, c3 = 0, c5 = 0 
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

%% DEFINING INITIAL AND FINAL POSE
% Define the variation of displacement in each direction
% dx = 0.0; % [m]
% dy = 0.0; % [m]
% dz = 0.2; % [m]
% delta = [dx; dy; dz];

% singularity at -0.23, 0.28. 0.89
R = 0.2;
gamma = -pi/2;

C = [p_sing(1);
     p_sing(2);
     p_sing(3) - R];

rep = 1;
freq = rep/T; 
%% DEFINING DESIRED TRAJECTORY 
t_in = 0; % initial time [s]
t_fin = t_in + T; % final time [s]

syms t_sym real
tau = t_sym / T;


s = 2* rep * pi * (6 * tau^5 - 15 * tau^4 + 10 * tau^3);

p_x_t = C(1);
p_y_t = C(2) + R * cos(s + gamma);
p_z_t = C(3) + R * sin(s + gamma);

p_d_sym = [p_x_t; p_y_t; p_z_t];

p_in = subs(p_d_sym, t_sym, t_in); % initial position at t = 0
p_fin = subs(p_d_sym, t_sym, t_fin); % final position at t = T
p_in = double(p_in); % convert to double
p_fin = double(p_fin); % convert to double

p_in = p_in + [-0.05; -0.05; -0.05];
q_in = num_IK_retry(p_in(1:3)); 
q_fin = num_IK_retry(p_fin(1:3));




R_in = [0, 1, 0;
        0, 0, 1;
        1, 0, 0];

R_fin = R_in; % const. orientation

phi_in = get_phi(R_in); % initial orientation angles
phi_fin = get_phi(R_fin); % final orientation angles

delta_phi = phi_fin - phi_in;
phi_d_sym = vpa(phi_in + delta_phi * (6 * tau^5 - 15 * tau^4 + 10 * tau^3)); % quintic polynomial

r_d_sym = vpa([p_d_sym; phi_d_sym])
r_dot_sym = diff(r_d_sym, t_sym);  % firt der
r_ddot_sym = diff(r_dot_sym, t_sym); % second der


% R_in = get_R(q_in);
% R_fin = get_R(q_fin);

% Compute the initial and final XYZ Euler orientation
% seq_rot = 'XYZ';
% phi_in = get_phi(R_in); % initial orientation angles
% phi_fin = get_phi(R_fin); % final orientation angles

% dp_sym = diff(p_d_sym, t_sym); % first derivative (velocity)
% ddp_sym = diff(dp_sym, t_sym); % second derivative (acceleration)


q_list = []; % to store joint positions
dq_list = []; % to store joint velocities
ddq_list = []; % to store joint accelerations
phi_list = []; % to store orientation angles
p_list = []; % to store end-effector positions
dp_list = []; % to store end-effector velocities
ddp_list = []; % to store end-effector accelerations
error_list = []; % to store error norms

qA_idx = [1,2,3,4,5,6]; % indices of joints in A (nonsingular)
qB_idx = [7]; % indices of joints in B (N-M = 1)
alpha = 1;
damp = 2;
use_RG = true; % use reduced gradient step if true, else use projected gradient step

dt = 0.01; % time step
t = 0.0;
q = q_in; % initialize joint position
dq = zeros(N, 1); % initialize joint velocity
ddq = zeros(N, 1); % initialize joint acceleration

t_sing = 0;

disp("press any key to start the simulation...");
pause; % wait for user input to start the simulation

while t <= t_fin % run for a fixed time


    %if t > T/2
    %    qA_idx = [2,3,4,5,6,7]; % indices of joints in A (nonsingular)
    %    qB_idx = [1]; % indices of joints in B (N-M = 1)
    %end

    % Nominal Trajectory
    p_nom = double(subs(r_d_sym, t_sym, t)); % expected end-effector position at time t
    dp_nom = double(subs(r_dot_sym, t_sym, t)); 
    ddp_nom = double(subs(r_ddot_sym, t_sym, t)); % expected end-effector acceleration at time t
    
    % LOGGING errors and pos
        p = get_p(q, true); % compute current end-effector position
        J = get_J(q, true);
        J_dot = get_J_dot(q, dq, true); % compute current end-effector Jacobian and its time derivative
        phi = get_phi(get_R(q)); % get the orientation angles from the rotation matrix

        min_singular = svds(J, 1, 'smallest');
        disp(['Minimum singular value of J: ', num2str(min_singular)]);

        dp = J * dq;
        ddp = J * ddq + J_dot * dq; % compute current end-effector acceleration
        error = p_nom(1:3) - p(1:3); % compute error in end-effector position
        norm_e = double(norm(error));
        detJJtrans = det(J*J');
        disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] dp = [', num2str(dp'), '] ddp = [', num2str(ddp_nom'), ']']);
        disp( ['q = [', num2str(q'), ']']);
        fprintf("det(J*J') = %.4f\n", detJJtrans);
        disp(['norm error = ', num2str(norm_e)]);

        error_list = [error_list, norm_e]; % store error norm for plotting later
        q_list = [q_list, q]; % store joint position
        dq_list = [dq_list, dq]; % store joint velocity
        ddq_list = [ddq_list, ddq]; % store joint acceleration
        p_list = [p_list, p]; % store end-effector position
        dp_list = [dp_list, dp]; % store end-effector velocity
        ddp_list = [ddp_list, ddp]; % store end-effector acceleration
        phi_list = [phi_list, phi]; % store orientation angles

    % [!] PG or RG step
    if use_RG == true
        ddq = reduced_grad_step_acc(q, dq, ddp_nom, qA_idx, qB_idx, p_nom, dp_nom, alpha, damp); % compute joint velocity using reduced gradient step
    else
        ddq = proj_grad_step_acc(q, dq, ddp_nom, p_nom, dp_nom); % compute joint acceleration using projected gradient step
    end
    disp(['ddq = [', num2str(ddq'), ']']);

    % CHECK Limits
    ddq = clamp_vec(ddq, -LIM_ddq_max, LIM_ddq_max); % clamp joint acceleration to max limits
    disp(['Clamped ddq = [', num2str(ddq'), ']']);

    % compute dq
    dq = dq + ddq * dt; % update joint velocity
    dq = clamp_vec(dq, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
    disp(['Clamped dq = [', num2str(dq'), ']']);

    q = q + dq * dt + 0.5*ddq*dt^2; % update joint position (integration step as in De Luca Paper)
    q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits

    % if we are near the singularity, we want to save the time in t_sing
    if norm(p(1:3)-p_sing) < 0.01 % if we are within 0.1 rad of the singularity
            t_sing = t; % save the time
            fprintf("Near singularity at t = %.2f s\n", t_sing);
    end

    t = t + dt; % update time 
end


fin_err = p_fin(1:3) - p(1:3);
fprintf("Norm of final error: %.4f\n", norm(fin_err))



% % plot joint over time
disp("Simulation finished. Plotting results...");

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
% plot joint accelerations with bounds for each joint
figure;
for i = 1:N
    subplot(N, 1, i);
    plot(time, ddq_list(i, :), 'b', 'DisplayName', ['q', num2str(i), ' Acceleration']);
    hold on;
    yline(LIM_ddq_max(i), 'r--', 'DisplayName', ['q', num2str(i), ' Max']);
    yline(-LIM_ddq_max(i), 'g--', 'DisplayName', ['q', num2str(i), ' Min']);
    xlabel('Time (s)');
    ylabel(['q', num2str(i), ' Acceleration (rad/s^2)']);
    title(['Joint ', num2str(i), ' Acceleration Over Time']);
    grid on;
    legend;
end
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

%plot euler angles (one plot for each angle, singulariy at +- pi/2 on phi2)
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(time, rad2deg(phi_list(i, :)), 'b', 'DisplayName', ['Real Phi', num2str(i)]);
    hold on;
    plot(time, rad2deg(double(subs(r_d_sym(i + 3), t_sym, time))), 'r--', 'DisplayName', ['Nominal Phi', num2str(i)]);
    if i == 2 % singularity at phi2 = +- pi/2
        yline(rad2deg(pi/2), 'r--', 'DisplayName', ['Phi', num2str(i), ' Max']);
        yline(rad2deg(-pi/2), 'g--', 'DisplayName', ['Phi', num2str(i), ' Min']);
    end
    xlabel('Time (s)');
    ylabel(['Phi', num2str(i), ' (deg)']);
    title(['Orientation Angle Phi', num2str(i), ' Over Time']);
    grid on;
    legend;
end



% After computing p_start and p_end, add this in the plotting section:
figure
plot3(p_list(1, :), p_list(2, :), p_list(3, :), 'b-', 'LineWidth', 2);
hold on;

% Sample the nominal trajectory over time
p_nominal_sampled = double(subs(p_d_sym, t_sym, time));
p_nom_in = p_nominal_sampled(:, 1);
p_nom_fin = p_nominal_sampled(:, end);

plot3(p_nominal_sampled(1, :), p_nominal_sampled(2, :), p_nominal_sampled(3, :), 'r--', 'LineWidth', 2);
scatter3(p_nom_in(1), p_nom_in(2), p_nom_in(3), 200, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r'); % Start point: blue circle
scatter3(p_nom_fin(1), p_nom_fin(2), p_nom_fin(3), 200, '^', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r'); % End point: blue upward triangle
scatter3(p_sing(1), p_sing(2), p_sing(3), 300, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 2); % Singularity point in red
scatter3(p_in(1), p_in(2), p_in(3), 200, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); % Start point: blue circle
scatter3(p_fin(1), p_fin(2), p_fin(3), 200, '^', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); % End point: blue upward triangle
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('End-Effector Position Over Time');
legend('Actual End-Effector Path', 'Nominal Trajectory', 'Singularity Point', 'Start Point', 'End Point', 'Nominal Start Point', 'Nominal End Point');

dx = R + 0.05;

xlim([C(1) - dx, C(1) + dx])
ylim([C(2) - dx, C(2) + dx])
zlim([C(3) - dx, C(3) + dx])

grid on;

% plot of the norm of the velocity
figure
v_nominal_sampled = double(subs(r_dot_sym(1:3), t_sym, time));
v_nom_norm = vecnorm(v_nominal_sampled);
v_ee_norm = vecnorm(dp_list(1:3,:));

plot(time, v_nom_norm, 'r--', 'LineWidth', 2);
hold on
plot(time, v_ee_norm, 'b-', 'LineWidth', 2);

xlabel('Time (s)');
ylabel('Velocity Norm (m/s)');
title('Velocity Magnitude Over Time');
legend('Nominal Velocity', 'End-Effector Velocity');

grid on;

% Acceleration
figure
a_nominal_sampled = double(subs(r_ddot_sym(1:3), t_sym, time));
a_nom_norm = vecnorm(a_nominal_sampled);
a_ee_norm = vecnorm(ddp_list(1:3,:));

plot(time, a_nom_norm, 'r--', 'LineWidth', 2);
hold on
plot(time, a_ee_norm, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration Norm (m/s^2)');
title('Acceleration Magnitude Over Time');
legend('Nominal Acceleration', 'End-Effector Acceleration');

grid on;