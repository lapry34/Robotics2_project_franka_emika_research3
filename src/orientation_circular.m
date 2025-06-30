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
T = 18; % total time for the trajectory [s]
dt = 0.001;
rep = 3;
R = 0.2; % radius of the circular path [m]
initial_offset = [-0.05; -0.05; -0.05]; % initial offset from the path
alpha = 1;
damp = 2;
use_RG = true; % use reduced gradient step if true, else use projected gradient step
use_accel = true; % use acceleration if true, else use velocity

% For RG only
list_joints = 1:N;
qB_idx1 = [3]; % indices of joints in B (N-M = 1)
qA_idx1 = setdiff(list_joints, qB_idx1);  % indices of joints in A (nonsingular)

qB_idx2 = [6];
qA_idx2 = setdiff(list_joints, qB_idx2);


if use_accel == true % when we are at acceleration level
    if use_RG == true
        Kp = 2;
        Kd = 2;
        alpha = 0.0075;
        damp = 2;
        qB_idx2 = qB_idx1;
        qA_idx2 = qA_idx1;
    else
        Kp = 5;
        Kd = 5;
    end

else % when we are at velocity level
    if use_RG == true
        Kp = 2;
    else
        Kp = 2;
    end
    Kd = 2;  % Not used for velocity (use_accel=false)
end

% Kp
% Kd
% alpha
% damp
% qA_idx1
% qB_idx1
% qA_idx2
% qB_idx2
 
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

%% DEFINING DESIRED TRAJECTORY 
gamma = -pi/2;

syms t_sym real
tau = t_sym / T;
s = 2* rep * pi * (6 * tau^5 - 15 * tau^4 + 10 * tau^3);
% singularity at -0.23, 0.28. 0.89
% Center of the circular path
C = [p_sing(1);
     p_sing(2);
     p_sing(3) - R];

freq = rep/T; 

t_in = 0; % initial time [s]
t_fin = t_in + T; % final time [s]


p_x_t = C(1) + R * cos(s + gamma);
p_y_t = C(2);
p_z_t = C(3) + R * sin(s + gamma);

p_d_sym = [p_x_t; p_y_t; p_z_t];

p_in = subs(p_d_sym, t_sym, t_in); % initial position at t = 0
p_fin = subs(p_d_sym, t_sym, t_fin); % final position at t = T
p_in = double(p_in); % convert to double
p_fin = double(p_fin); % convert to double

p_in = p_in + initial_offset;
q_in = num_IK_retry(p_in(1:3)); 
q_fin = num_IK_retry(p_fin(1:3));

% Initlai orientation (constant)
R_in = [0, 1, 0;
        0, 0, 1;
        1, 0, 0];

R_fin = R_in; % const. orientation

phi_in = get_phi(R_in); % initial orientation angles
phi_fin = get_phi(R_fin); % final orientation angles
disp(['phi_in = ', num2str(rad2deg(phi_in'))]);
disp(['phi_fin = ', num2str(rad2deg(phi_fin'))]);

delta_phi = phi_fin - phi_in;
phi_d_sym = vpa(phi_in + delta_phi * (6 * tau^5 - 15 * tau^4 + 10 * tau^3)); % quintic polynomial

% Defining the task
r_d_sym = vpa([p_d_sym; phi_d_sym]);
r_dot_sym = diff(r_d_sym, t_sym);  % firt der
r_ddot_sym = diff(r_dot_sym, t_sym); % second der

% Fing the t when the manipulator is expected to pass in singularity
eq1 = s == pi;
t_sing_1 = double(solve(eq1, t_sym));
eq2 = s == 3*pi;
t_sing_2 = double(solve(eq2, t_sym));
eq3 = s == 5*pi;
t_sing_3 = double(solve(eq3, t_sym));

%% ALGORITHM
q_list = []; % to store joint positions
dq_list = []; % to store joint velocities
ddq_list = []; % to store joint accelerations
phi_list = []; % to store orientation angles
p_list = []; % to store end-effector positions
dp_list = []; % to store end-effector velocities
ddp_list = []; % to store end-effector accelerations
error_list = []; % to store error norms
H_man_list = []; % to store manipulability measure

 % time step
t = 0.0;
q = q_in; % initialize joint position
dq = zeros(N, 1); % initialize joint velocity
ddq = zeros(N, 1); % initialize joint acceleration

t_sing = 0;

if ~use_accel
    disp("Using velocity control (no acceleration)");
else
    disp("Using acceleration control");
end
if use_RG
    disp("Using Reduced Gradient Step");
else
    disp("Using Projected Gradient Step");
end

disp("press any key to start the simulation...");
pause; % wait for user input to start the simulation

while t <= t_fin % run for a fixed time

    % Switch indices based on which half of each repetition we're in
    rep_progress = mod(t * freq, 1); % progress within current repetition (0 to 1)

    if rep_progress < 0.5
        qA_idx = qA_idx1; % indices of joints in A (nonsingular)
        qB_idx = qB_idx1; % indices of joints in B (N-M = 1)
    else
        qA_idx = qA_idx2; % indices of joints in A (nonsingular)
        qB_idx = qB_idx2; % indices of joints in B (N-M = 1)
    end

    % Nominal Trajectory
    p_nom = double(subs(r_d_sym, t_sym, t)); % expected end-effector position at time t
    dp_nom = double(subs(r_dot_sym, t_sym, t)); 
    ddp_nom = double(subs(r_ddot_sym, t_sym, t)); % expected end-effector acceleration at time t
    
    % LOGGING errors and pos
        p = get_p(q, true); % compute current end-effector position
        J = get_J(q, true);
        J_dot = get_J_dot(q, dq, true); % compute current end-effector Jacobian and its time derivative
        phi = get_phi(get_R(q)); % get the orientation angles from the rotation matrix

        %min_singular = svds(J, 1, 'smallest');
        %disp(['Minimum singular value of J: ', num2str(min_singular)]);

        dp = J * dq;
        ddp = J * ddq + J_dot * dq; % compute current end-effector acceleration
        error = p_nom(1:3) - p(1:3); % compute error in end-effector position
        norm_e = double(norm(error));
        detJJtrans = det(J*J');
        H_man = sqrt(detJJtrans); % manipulability measure
        %disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] dp = [', num2str(dp'), '] ddp = [', num2str(ddp_nom'), ']']);
        %disp( ['q = [', num2str(q'), ']']);
        %fprintf("det(J*J') = %.4f\n", detJJtrans);
        %disp(['norm error = ', num2str(norm_e)]);

        error_list = [error_list, norm_e]; % store error norm for plotting later
        q_list = [q_list, q]; % store joint position
        dq_list = [dq_list, dq]; % store joint velocity
        ddq_list = [ddq_list, ddq]; % store joint acceleration
        p_list = [p_list, p]; % store end-effector position
        dp_list = [dp_list, dp]; % store end-effector velocity
        ddp_list = [ddp_list, ddp]; % store end-effector acceleration
        phi_list = [phi_list, phi]; % store orientation angles
        H_man_list = [H_man_list, H_man]; % store manipulability measure
    % [!] PG or RG step
    if use_accel == true
        if use_RG == true
            ddq = reduced_grad_step_acc(q, dq, ddp_nom, qA_idx, qB_idx, p_nom, dp_nom, alpha, damp,Kp,Kd); % compute joint velocity using reduced gradient step
        else
            ddq = proj_grad_step_acc(q, dq, ddp_nom, p_nom, dp_nom, Kp, Kd); % compute joint acceleration using projected gradient step
        end
        %disp(['ddq = [', num2str(ddq'), ']']);

        % CHECK Limits
        ddq = clamp_vec(ddq, -LIM_ddq_max, LIM_ddq_max); % clamp joint acceleration to max limits
        %disp(['Clamped ddq = [', num2str(ddq'), ']']);

        % compute dq
        dq = dq + ddq * dt; % update joint velocity
        %disp(['Clamped dq = [', num2str(dq'), ']']);
    else
        if use_RG == true
            dq = reduced_grad_step(q, dp_nom, qA_idx, qB_idx, p_nom,Kp); % compute joint velocity using reduced gradient step
        else
            dq = proj_grad_step(q, dp_nom, p_nom, Kp); % compute joint velocity using projected gradient step
        end
    end
    dq = clamp_vec(dq, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
    q = q + dq * dt + 0.5*ddq*dt^2; % update joint position (integration step as in De Luca Paper)
    q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits

    % if we are near the singularity, we want to save the time in t_sing
    if norm(p(1:3)-p_sing) < 0.01 % if we are within 0.1 rad of the singularity
            t_sing = t; % save the time
            fprintf("Near singularity at t = %.2f s\n", t_sing);
    end

    t = t + dt; % update time 
    fprintf("t = %.2f s\n", t);
end


fin_err = p_fin(1:3) - p(1:3);
fprintf("Norm of final error: %.4f\n", norm(fin_err))


%% PLOT OF THE RESULTS
% plot joint over time
disp("Simulation finished. Plotting results...");
time = 0:dt:t_fin;
min_len = min(length(time), length(q_list));
time = time(1:min_len);

t_sing = [t_sing_1, t_sing_2, t_sing_3];

if use_accel == true
    plot_all(   N, T, ...
            dt, t_fin, t_sym, t_sing, ...
            p_list, p_in, p_fin, p_sing, r_d_sym, ...
            q_list, dq_list, error_list, ...
            LIM_dq_max, LIM_q_max, LIM_q_min, ...
            2, ... % want_acc_orient = 1 (plot accelerations)
            ddq_list, phi_list, r_d_sym, H_man_list ...
            )
else 
    plot_all(   N, T, ...
                dt, t_fin, t_sym, t_sing, ...
                p_list, p_in, p_fin, p_sing, r_d_sym, ...
                q_list, dq_list, error_list, ...
                LIM_dq_max, LIM_q_max, LIM_q_min, ...
                2, ... % want_acc_orient = 2 (plot orientations)
                [], ... % ddq_list not needed for orientation plot
                phi_list, r_d_sym, H_man_list ...
                )
end

plotNormVelocity(time, dp_list, r_dot_sym, t_sym)
plotNormAcceleration(time, ddp_list, r_ddot_sym, t_sym)

%%

% After computing p_start and p_end, add this in the plotting section:
fig = figure;
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
exportgraphics(fig, 'EndEffector3D.png', 'Resolution', 300);

close all;
%% Moving the figures

if use_RG == true
    method = "RG";
else
    method = "PG";
end

if use_accel == true
    acc_flag = "_acceleration";
else 
    acc_flag = "";
end

% Usa filesep per compatibilità multipiattaforma
save_imgs_path = fullfile("figures", method + "_circular" + acc_flag);

if ~exist(save_imgs_path, 'dir')
    mkdir(save_imgs_path);
end

png_files = dir("*.png");

for k = 1:length(png_files)
    source_file = png_files(k).name;
    movefile(source_file, fullfile(save_imgs_path, source_file));
end

