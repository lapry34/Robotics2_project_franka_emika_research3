clc
clear all
digits 4
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")
addpath("./Trajectory/")
addpath("./Plots/")

% GLOBALS
N = 7; % number of joints
T = 3; % total time for the trajectory [s]


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
q_start = num_IK_retry(p_start); % compute inverse kinematics for the start position
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


dt = 0.001; % time step
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
    
    % LOGGING errors and pos
        p = get_p(q); % compute current end-effector position
        J = get_J(q);

        min_singular = svds(J, 1, 'smallest');
        disp(['Minimum singular value of J: ', num2str(min_singular)]);

        dp = J * dq;
        error = dp_nom - dp;
        norm_e = double(norm(error));
        detJJtrans = det(J*J');
        disp (['t = ', num2str(t), ' s, p = [', num2str(p'), '] dp = [', num2str(dp'), ']']);
        disp( ['q = [', num2str(q'), ']']);
        fprintf("det(J*J') = %.4f\n", detJJtrans);
        disp(['norm error = ', num2str(norm_e)]);

        error_list = [error_list, norm_e]; % store error norm for plotting later
        q_list = [q_list, q]; % store joint position
        dq_list = [dq_list, dq]; % store joint velocity
        p_list = [p_list, p]; % store end-effector position
    
    % [!] PG step
    dq = proj_grad_step(q, dp_nom, p_nom, 4); % compute joint velocity using projected gradient step
    disp(['dq = [', num2str(dq'), ']']);

    % CHECK Limits
    dq = clamp_vec(dq, -LIM_dq_max, LIM_dq_max); % clamp joint velocity to max limits
    disp(['Clamped dq = [', num2str(dq'), ']']);

    q = q + dq * dt; % update joint position
    q = clamp_vec(q, LIM_q_min, LIM_q_max); % clamp joint position to limits

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

plot_all(   N, T, ...
            dt, t_fin, t_sym, t_sing, ...
            p_list, p_start, p_end, p_sing, p_d_sym, ...
            q_list, dq_list, error_list, ...
            LIM_dq_max, LIM_q_max, LIM_q_min ...
)
