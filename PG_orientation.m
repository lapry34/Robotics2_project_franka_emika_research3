clc
clear
close all
digits 4
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")
addpath("./Trajectory/")

%% GLOBALS
N = 7; % number of joints
T = 3; % total time for the trajectory [s]

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
dx = 0.0; % [m]
dy = 0.0; % [m]
dz = 0.2; % [m]
delta = [dx; dy; dz];

% singularity at -0.23, 0.28. 0.89
p_in = p_sing;
p_in = p_in - delta/2;

% Setting p_d_end (desired)
p_fin = p_sing;
p_fin = p_fin + delta/2;

% Compute inverse kinematics for the initial and final position
% TODO: forse dobbiamo cambiare questo. In realtà le rotazioni inizali e
% finali le scelgiamo noi.

q_in = num_IK(p_in(1:3)); 
q_fin = num_IK(p_fin(1:3));

R_in = get_R(q_in);
R_fin = get_R(q_fin);

% Compute the initial and final XYZ Euler orientation
seq_rot = 'XYZ';
phi_in = get_phi(R_in); % initial orientation angles
phi_fin = get_phi(R_fin); % final orientation angles

r_in = [p_in; phi_in];
r_fin = [p_fin; phi_fin];


%% DEFINING DESIRED TRAJECTORY 
t_in = 0; % initial time [s]
t_fin = t_in + T; % final time [s]

% quintic rest-to-rest
syms t_sym real
tau = t_sym/T;

delta_r = r_fin - r_in;
r_d_sym = vpa(r_in + delta_r * (6 * tau^5 - 15 * tau^4 + 10 * tau^3)); % quintic polynomial
r_dot_sym = diff(r_d_sym, t_sym);  % firt der

%% DEFINING INITIAL JOINT CONFIGURATION
% Adding some ERROR
% we set an amount of error for the controller to recover
q_in(1) = q_in(1)/2; 
q_in(2) = q_in(2)/2;
%% PROJECTED GRADIENT
% Init of useful list
q_list = []; % to store joint positions
dq_list = []; % to store joint velocities
phi_list = []; % to store orientation angles
p_list = []; % to store end-effector positions
error_list = []; % to store error norms
 
dt = 0.01; % time step
t = 0.0;
q = q_in; % initialize joint position
q_dot = zeros(N, 1); % initialize joint velocity

t_sing = 0;

disp("Simulation values...");
disp(['Start position: p_in = [', num2str(p_in'), ']']);
disp(['Singularity position: p_sing = [', num2str(p_sing'), ']']);
disp(['End position: p_fin = [', num2str(p_fin'), ']']);
disp(['Start joint angles: q_start = [', num2str(q_in'), ']']);
disp(['Time step: dt = ', num2str(dt), ' s']);
disp(['Total simulation time: t_fin = ', num2str(t_fin), ' s']);

disp("Press enter to start the simulation...");
pause; % wait for user input to start the simulation

print_info = false;
while t < t_fin % run for a fixed time

    % Nominal Trajectory
    r_d_nom = double(subs(r_d_sym, t_sym, t)); % expected end-effector pose at time t
    r_dot_nom = double(subs(r_dot_sym, t_sym, t)); 

    % LOGGING errors and pos
    p = get_p(q, true); % compute current end-effector pose
    J = get_J(q, true);
    phi = get_phi(get_R(q)); % get the orientation angles from the rotation matrix
 
    r_dot = J * q_dot;
%     error = r_dot_nom - r_dot;
    error = r_d_nom - p;    % position error
    norm_e = double(norm(error));
    detJJtrans = det(J*J');
    
    if print_info == true
        min_singular = svds(J, 1, 'smallest');
        disp(['Minimum singular value of J: ', num2str(min_singular)]);
        disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] p_dot = [', num2str(r_dot'), ']']);
        disp( ['q = [', num2str(q'), ']']);
        fprintf("det(J*J') = %.4f\n", detJJtrans);
        disp(['norm error = ', num2str(norm_e)]);
    end

    error_list = [error_list, norm_e]; % store error norm for plotting later
    q_list = [q_list, q]; % store joint position
    dq_list = [dq_list, q_dot]; % store joint velocity
    p_list = [p_list, p]; % store end-effector position
    phi_list = [phi_list, phi]; % store orientation angles
    % [!] PG step
    q_dot = proj_grad_step(q, r_dot_nom, r_d_nom); % compute joint velocity using projected gradient step
    q_dot = double(q_dot);

    if print_info == true
        disp(['q_dot = [', num2str(q_dot'), ']']);
    end

    % CHECK Limits
%     q_dot = clamp_vec(q_dot, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
%     if print_info == true
%         disp(['Clamped dq = [', num2str(q_dot'), ']']);
%     end

    q = q + q_dot * dt; % update joint position
 
%     q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits


    % if we are near the singularity, we want to save the time in t_sing
    if norm(p(1:3) - p_sing) < 0.01 % if we are within 0.1 rad of the singularity
            t_sing = t; % save the time
            if print_info==true
                fprintf("Near singularity at t = %.2f s\n", t_sing);
            end
    end

    t = t + dt; % update time 
end

fin_err = p_fin - p(1:3);
fprintf("Norm of final error: %.4f\n", norm(fin_err))

%% PLOT OF THE RESULTS
% plot joint over time
disp("Simulation finished. Plotting results...");

figure;

% Plot end-effector position over time
subplot(2, 1, 1);
hold on;

% time = 0:dt:t_fin-dt;
% if T > 2
%     time = 0:dt:t_fin; % time vector for plotting
% else
%     time = 0:dt:t_fin-dt; % time vector for plotting
% end
time = 0:dt:t_fin;


plot(time, p_list(1, :), 'b', 'DisplayName', 'Real Position (X)');
plot(time, double(subs(r_d_sym(1), t_sym, time)), 'r--', 'DisplayName', 'Nominal Position (X)');
plot(time, p_list(2, :), 'g', 'DisplayName', 'Real Position (Y)');
plot(time, double(subs(r_d_sym(2), t_sym, time)), 'm--', 'DisplayName', 'Nominal Position (Y)');
plot(time, p_list(3, :), 'c', 'DisplayName', 'Real Position (Z)');
plot(time, double(subs(r_d_sym(3), t_sym, time)), 'k--', 'DisplayName', 'Nominal Position (Z)');
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position Over Time (X, Y, Z)');
legend;
grid on;
%%
% Plot norm of the error over time
subplot(2, 1, 2);
plot(time, error_list, 'g', 'DisplayName', 'Norm of Error');
xlabel('Time (s)');
ylabel('Error Norm (m)');
title('Norm of End-Effector Position Error Over Time');
xline(t_sing, '--r', 'Singularity Crossing', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle', 'HandleVisibility', 'off');
legend;
grid on;
%%
% Plot joint positions
figure;
plot(time, q_list);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Time');
% Add grid and legend
grid on;
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
%%

%plot euler angles (one plot for each angle, singulariy at +- pi/2 on phi2)
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(time, phi_list(i, :), 'b', 'DisplayName', ['Phi', num2str(i)]);
    hold on;
    if i == 2 % singularity at phi2 = +- pi/2
        yline(pi/2, 'r--', 'DisplayName', ['Phi', num2str(i), ' Max']);
        yline(-pi/2, 'g--', 'DisplayName', ['Phi', num2str(i), ' Min']);
    end
    xlabel('Time (s)');
    ylabel(['Phi', num2str(i), ' (rad)']);
    title(['Orientation Angle Phi', num2str(i), ' Over Time']);
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
%%
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

%%

% After computing p_in and p_fin, add this in the plotting section:
figure
plot3(p_list(1, :), p_list(2, :), p_list(3, :));
hold on;
plot3([p_in(1) p_fin(1)], [p_in(2) p_fin(2)], [p_in(3) p_fin(3)], 'g--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('End-Effector Position Over Time');
scatter3(p_sing(1), p_sing(2), p_sing(3), 50, 'filled', 'MarkerFaceColor', 'k'); % Singularity point in red
scatter3(p_in(1), p_in(2), p_in(3), 10, 'filled', 'MarkerFaceColor', 'g'); % Start point in green
scatter3(p_fin(1), p_fin(2), p_fin(3), 10, 'filled', 'MarkerFaceColor', 'b'); % End point in blue
legend('End-Effector Path', 'Start to End Path', 'Singularity Point', 'Start Point', 'End Point');

dx = 0.5;

xlim([p_in(1) - dx, p_in(1) + dx])
ylim([p_in(2) - dx, p_in(2) + dx])
zlim([p_in(3) - dx, p_in(3) + dx])
grid on;