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

q_in = num_IK_retry(p_in(1:3)); 
q_fin = num_IK_retry(p_fin(1:3));

% R_in = get_R(q_in);
% R_fin = get_R(q_fin);
% Compute the initial and final XYZ Euler orientation
seq_rot = 'XYZ';
% phi_in = get_phi(R_in); % initial orientation angles
% phi_fin = get_phi(R_fin); % final orientation angles

phi_in = [3.1045, 0.5075, 0.1272]';
phi_fin = [1.8823, 0.7, 1.29789]';

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
r_ddot_sym = diff(r_dot_sym, t_sym); % second der

%% DEFINING INITIAL JOINT CONFIGURATION
% Adding some ERROR
% we set an amount of error for the controller to recover
q_in(1) = q_in(1)/2; 
q_in(2) = q_in(2)/2;
%% PROJECTED GRADIENT
% Init of useful list
q_list = []; % to store joint positions
dq_list = []; % to store joint velocities
ddq_list = []; % to store joint accelerations
phi_list = []; % to store orientation angles
p_list = []; % to store end-effector positions
error_list = []; % to store error norms
 
dt = 0.001; % time step
t = 0.0;
q = q_in; % initialize joint position
q_dot = zeros(N, 1); % initialize joint velocity
q_ddot = zeros(N, 1); % initialize joint acceleration

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
    r_ddot_nom = double(subs(r_ddot_sym, t_sym, t)); % expected end-effector acceleration at time t

    % LOGGING errors and pos
    p = get_p(q, true); % compute current end-effector pose
    J = get_J(q, true);
    J_dot = get_J_dot(q, q_dot, true); % compute current end-effector Jacobian and its time derivative
    phi = get_phi(get_R(q)); % get the orientation angles from the rotation matrix
 
    r_dot = J * q_dot;
    r_ddot = J * q_ddot + J_dot * q_dot; % compute current end-effector acceleration
    error = r_d_nom(1:3) - p(1:3);     % position error
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
    ddq_list = [ddq_list, q_ddot]; % store joint acceleration
    p_list = [p_list, p]; % store end-effector position
    phi_list = [phi_list, phi]; % store orientation angles
    % [!] PG step
    q_ddot = proj_grad_step_acc(q, q_dot, r_ddot_nom, r_d_nom, r_dot_nom, 10, 5);
    q_ddot = double(q_ddot);

    if print_info == true
        disp(['q_ddot = [', num2str(q_ddot'), ']']);
    end

    % CHECK Limits
    q_ddot = clamp_vec(q_ddot, -LIM_ddq_max, LIM_ddq_max); % clamp joint acceleration to max limits
    if print_info == true
        disp(['Clamped q_ddot = [', num2str(q_ddot'), ']']);
    end

    q_dot = q_dot + q_ddot * dt; % update joint velocity
    if print_info == true
        disp(['q_dot = [', num2str(q_dot'), ']']);
    end

     q_dot = clamp_vec(q_dot, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
     if print_info == true
         disp(['Clamped dq = [', num2str(q_dot'), ']']);
     end

    q = q + q_dot * dt + 0.5*q_ddot*dt^2; % update joint position (integration step as in De Luca Paper)
 
    q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits


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

plot_all(   N, T, ...
            dt, t_fin, t_sym, t_sing, ...
            p_list, p_in, p_fin, p_sing, r_d_sym, ...
            q_list, dq_list, error_list, ...
            LIM_dq_max, LIM_q_max, LIM_q_min, ...
            3, ... % want_acc_orient = 3 (plot accelerations and orientations)
            ddq_list, ...
            phi_list, r_d_sym ...
)
%% Moving the figures
save_imgs_path = "figures\PG_orientation_acceleration\";
if ~exist(save_imgs_path, 'dir')
    mkdir(save_imgs_path);
end

png_files = dir("*.png");

for k = 1:length(png_files)
    source_file = png_files(k).name;
    movefile(source_file, fullfile(save_imgs_path, source_file));
end