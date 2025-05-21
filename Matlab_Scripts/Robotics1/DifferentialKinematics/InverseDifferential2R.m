clear variables;
clc;

% Define symbolic variables
syms q1 q2 real
l1 = 2;  % Length of link 1
l2 = 2;  % Length of link 2

% Define the direct kinematics function
f = @(q) [l1*cos(q(1)) + l2*cos(q(1) + q(2)); l1*sin(q(1)) + l2*sin(q(1) + q(2))]; %2R robot

% Compute the Jacobian symbolically
J = @(q) [-l1*sin(q(1)) - l2*sin(q(1) + q(2)), -l2*sin(q(1) + q(2));
           l1*cos(q(1)) + l2*cos(q(1) + q(2)),  l2*cos(q(1) + q(2))];


% Desired end-effector trajectory
num_steps = 1000;  % Number of steps
q0 = [0; 0];  % Initial joint angles
qf = [pi/2; pi/2];  % Final joint angles

% Generate linearly spaced trajectory for the end-effector
q_traj = [linspace(q0(1), qf(1), num_steps);
          linspace(q0(2), qf(2), num_steps)];

% Time step
dt = 0.005;

% Storage for joint angles and end-effector positions
q_history = zeros(2, num_steps);
x_history = zeros(2, num_steps);

% Simulate the motion
q = q0;
for t = 1:num_steps
    % Compute the current end-effector position
    x_current = f(q);

    % Compute the desired end-effector position
    x_desired = [l1 * cos(q_traj(1, t)) + l2 * cos(q_traj(1, t) + q_traj(2, t));
                 l1 * sin(q_traj(1, t)) + l2 * sin(q_traj(1, t) + q_traj(2, t))];

    % Compute the velocity to move towards the desired position
    xdot = (x_desired - x_current) / dt;

    % Compute the Jacobian at the current joint angles
    J_curr = J(q);

    % Solve for joint velocities
    qdot = pinv(J_curr) * xdot;

    % Update joint angles
    q = q + qdot * dt;

    % Store the joint angles and end-effector positions
    q_history(:, t) = q;
    x_history(:, t) = x_current;
end

% Plot the joint angles over time
figure;
subplot(2, 1, 1);
plot((0:num_steps-1)*dt, q_history(1, :));
title('Joint Angle q_1 over Time');
xlabel('Time (s)');
ylabel('q_1 (rad)');

subplot(2, 1, 2);
plot((0:num_steps-1)*dt, q_history(2, :));
title('Joint Angle q_2 over Time');
xlabel('Time (s)');
ylabel('q_2 (rad)');

% Plot the end-effector trajectory
figure;
plot(x_history(1, :), x_history(2, :), 'b--', 'LineWidth', 1.5);
hold on;
title('End-Effector Trajectory');
xlabel('x (m)');
ylabel('y (m)');
legend('Trajectory');
grid on;
