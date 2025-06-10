% From https://frankaemika.github.io/docs

clear; clc; close all;
digits 4
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

addpath('./Matlab_Scripts/Robotics1');
% 1.  SYMBOLIC JACOBIAN  ---------------------------------------------------
% https://frankaemika.github.io/docs/control_parameters.html

theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]';
N = 7;
joints_str = 'RRRRRRR';

% limits-for-franka-research-3
Q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]'; % [rad]
Q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]'; % [rad]
Q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
Q_ddot_max = 10 * ones(1, N); % [rad/s^2]
Q_dddot_max = 5000 * ones(1, N);



%a_i = [0; 0; 0; a4; a5; 0; a7; 0];
%d_i = [d1; 0; d3; 0; d5; 0; 0; d_e];
alpha_i = [0; -pi/2; pi/2; pi/2; -pi/2; pi/2; pi/2; 0];
a_i = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
d_i = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
theta_i = [theta; 0];

DH_table = [alpha_i, a_i, d_i, theta_i]

T_ee = DHMatrix(DH_table);
p_ee = affine_get_translation(T_ee);
R_0_7 = affine_get_R(T_ee);

% geometric jacobian expressed in RF_0
[Jl, Ja] = geometric_jacobian(p_ee, joints_str, theta, DH_table);
J_geom = [Jl; Ja];
J_geom = simplify(J_geom);
% geometric jacobian expressed in RF_7
R_7_0 = R_0_7'; % orientation of RF0 w.r.t. RF7

R_j = blkdiag(R_7_0, R_7_0);
J_7 = simplify(R_j * J_geom)

%% Continue from symbolic J_7 to find singularities numerically
% This script continues from your symbolic Jacobian J_7

% Convert symbolic J_7 to a numerical function
fprintf('Converting symbolic Jacobian to numerical function...\n');
J_7_func = matlabFunction(J_7, 'Vars', {theta1, theta2, theta3, theta4, theta5, theta6, theta7});

% Create a wrapper function that takes a vector input
compute_J7 = @(q) J_7_func(q(1), q(2), q(3), q(4), q(5), q(6), q(7));

% Objective functions for finding singularities
% Method 1: Minimize smallest singular value
objective_svd = @(q) min(svd(compute_J7(q)));

% Method 2: Minimize determinant of J*J' (more robust for 6x7 matrix)
objective_det = @(q) sqrt(abs(det(compute_J7(q) * compute_J7(q)')));

% Method 3: Minimize reciprocal of condition number
objective_cond = @(q) 1/cond(compute_J7(q));

%% Find singularities using multi-start optimization
fprintf('\nFinding singularities using multi-start optimization...\n\n');

% Storage for found singularities
singularities = [];
singularity_types = {};
tolerance = 1e-6;

% Options for optimization
options = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'interior-point', ...
    'TolFun', 1e-12, ...
    'TolX', 1e-12, ...
    'MaxFunctionEvaluations', 10000);

% Number of random starts
n_starts = 200;

for i = 1:n_starts
    % Random starting point
    q0 = Q_min + (Q_max - Q_min) .* rand(7, 1);
    
    try
        % Find minimum using SVD criterion
        [q_sing, fval] = fmincon(objective_svd, q0, [], [], [], [], Q_min, Q_max, [], options);
        
        % Verify this is a true singularity
        J = compute_J7(q_sing);
        sv = svd(J);
        min_sv = min(sv);
        
        if min_sv < tolerance
            % Check if this is a new singularity
            is_new = true;
            for j = 1:size(singularities, 2)
                if norm(q_sing - singularities(:, j)) < 0.1
                    is_new = false;
                    break;
                end
            end
            
            if is_new
                singularities = [singularities, q_sing];
                fprintf('Found singularity %d: min(svd) = %.2e\n', size(singularities, 2), min_sv);
                fprintf('Joint angles [rad]: [');
                fprintf('%.4f ', q_sing);
                fprintf(']\n');
                fprintf('Joint angles [deg]: [');
                fprintf('%.1f ', q_sing * 180/pi);
                fprintf(']\n\n');
                
                % Classify singularity type
                type = classify_singularity(q_sing, Q_min, Q_max);
                singularity_types{end+1} = type;
            end
        end
    catch ME
        % Skip failed optimizations
        if mod(i, 50) == 0
            fprintf('Progress: %d/%d starts completed\n', i, n_starts);
        end
    end
end

%% Search for specific known singularity configurations
fprintf('\nSearching for specific singularity types...\n\n');

% 1. Elbow singularities (theta4 at limits)
fprintf('Checking elbow singularities...\n');
for theta4_val = [Q_min(4), Q_max(4)]
    % Fix theta4 and optimize other joints
    q_test = zeros(7, 1);
    q_test(4) = theta4_val;
    
    objective_fixed = @(q_reduced) objective_svd([q_reduced(1:3); theta4_val; q_reduced(4:6)]);
    q0_reduced = [zeros(3, 1); zeros(3, 1)];
    
    [q_opt, fval] = fmincon(objective_fixed, q0_reduced, [], [], [], [], ...
                            [Q_min([1:3, 5:7])], [Q_max([1:3, 5:7])], [], options);
    
    q_full = [q_opt(1:3); theta4_val; q_opt(4:6)];
    
    if fval < tolerance
        fprintf('  Elbow singularity at theta4 = %.4f rad (%.1f deg): min(svd) = %.2e\n', ...
                theta4_val, theta4_val * 180/pi, fval);
        
        % Add to singularities if new
        is_new = true;
        for j = 1:size(singularities, 2)
            if norm(q_full - singularities(:, j)) < 0.1
                is_new = false;
                break;
            end
        end
        if is_new
            singularities = [singularities, q_full];
            singularity_types{end+1} = 'Elbow';
        end
    end
end

% 2. Shoulder singularities (specific configurations)
fprintf('\nChecking shoulder singularities...\n');
% When the arm is fully extended in certain directions
test_configs = [
    [0; -pi/2; 0; -pi/2; 0; pi/2; 0];  % Stretched out
    [0; 0; 0; -pi/2; 0; 0; 0];         % Another common config
];

for k = 1:size(test_configs, 2)
    q_init = test_configs(:, k);
    [q_opt, fval] = fmincon(objective_svd, q_init, [], [], [], [], Q_min, Q_max, [], options);
    
    if fval < tolerance
        fprintf('  Shoulder singularity found: min(svd) = %.2e\n', fval);
        % Add if new...
    end
end

% 3. Wrist singularities (theta5 near 0)
fprintf('\nChecking wrist singularities...\n');
theta5_vals = [-0.1, 0, 0.1];  % Near zero
for theta5_val = theta5_vals
    if theta5_val >= Q_min(5) && theta5_val <= Q_max(5)
        q_test = zeros(7, 1);
        q_test(5) = theta5_val;
        
        objective_fixed = @(q_reduced) objective_svd([q_reduced(1:4); theta5_val; q_reduced(5:6)]);
        q0_reduced = [zeros(4, 1); zeros(2, 1)];
        
        [q_opt, fval] = fmincon(objective_fixed, q0_reduced, [], [], [], [], ...
                                [Q_min([1:4, 6:7])], [Q_max([1:4, 6:7])], [], options);
        
        q_full = [q_opt(1:4); theta5_val; q_opt(5:6)];
        
        if fval < tolerance
            fprintf('  Wrist singularity at theta5 = %.4f rad (%.1f deg): min(svd) = %.2e\n', ...
                    theta5_val, theta5_val * 180/pi, fval);
        end
    end
end

%% Detailed analysis of found singularities
fprintf('\n\n========== SINGULARITY ANALYSIS ==========\n');
fprintf('Total unique singularities found: %d\n\n', size(singularities, 2));

for i = 1:size(singularities, 2)
    q = singularities(:, i);
    J = compute_J7(q);
    [U, S, V] = svd(J);
    sv = diag(S);
    
    fprintf('Singularity %d', i);
    if i <= length(singularity_types) && ~isempty(singularity_types{i})
        fprintf(' (Type: %s)', singularity_types{i});
    end
    fprintf(':\n');
    
    fprintf('  Joint configuration:\n');
    fprintf('    [rad]: [');
    fprintf('%.4f ', q);
    fprintf(']\n');
    fprintf('    [deg]: [');
    fprintf('%.1f ', q * 180/pi);
    fprintf(']\n');
    
    fprintf('  Jacobian properties:\n');
    fprintf('    Singular values: [');
    fprintf('%.3e ', sv);
    fprintf(']\n');
    fprintf('    Rank: %d (should be < 6 for singularity)\n', rank(J, 1e-6));
    fprintf('    Condition number: %.2e\n', cond(J));
    
    % Find null space direction
    null_space = V(:, find(sv < 1e-6));
    if ~isempty(null_space)
        fprintf('    Null space dimension: %d\n', size(null_space, 2));
        fprintf('    Primary null space direction: [');
        fprintf('%.3f ', null_space(:, 1));
        fprintf(']\n');
    end
    
    fprintf('\n');
end

%% Create singularity maps
fprintf('\nCreating singularity visualization...\n');

% 1. 2D slice through joint space (theta1 vs theta4)
n_grid = 100;
theta1_range = linspace(Q_min(1), Q_max(1), n_grid);
theta4_range = linspace(Q_min(4), Q_max(4), n_grid);
[Theta1, Theta4] = meshgrid(theta1_range, theta4_range);

% Fixed values for other joints (home position)
q_nominal = zeros(7, 1);

fprintf('Computing singularity map...\n');
Manipulability = zeros(n_grid, n_grid);

for i = 1:n_grid
    for j = 1:n_grid
        q = q_nominal;
        q(1) = Theta1(i, j);
        q(4) = Theta4(i, j);
        
        J = compute_J7(q);
        % Yoshikawa's manipulability measure
        Manipulability(i, j) = sqrt(det(J * J'));
    end
    if mod(i, 10) == 0
        fprintf('  Progress: %d%%\n', round(i/n_grid*100));
    end
end

% Plot manipulability map
figure('Name', 'Franka Robot Singularity Map');
subplot(2, 1, 1);
imagesc(theta1_range*180/pi, theta4_range*180/pi, log10(Manipulability + 1e-10));
colorbar;
xlabel('\theta_1 [deg]');
ylabel('\theta_4 [deg]');
title('Manipulability Map (log_{10} scale)');
set(gca, 'YDir', 'normal');
hold on;

% Mark found singularities on the map
for i = 1:size(singularities, 2)
    plot(singularities(1, i)*180/pi, singularities(4, i)*180/pi, 'r*', 'MarkerSize', 10);
end

% 2. Plot singular value evolution along a path
subplot(2, 1, 2);
% Create a path from home to a singularity
if size(singularities, 2) > 0
    q_start = zeros(7, 1);
    q_end = singularities(:, 1);
    
    t = linspace(0, 1, 100);
    sv_evolution = zeros(6, length(t));
    
    for i = 1:length(t)
        q = q_start + t(i) * (q_end - q_start);
        J = compute_J7(q);
        sv_evolution(:, i) = svd(J);
    end
    
    plot(t, sv_evolution);
    xlabel('Path parameter');
    ylabel('Singular values');
    title('Singular values along path to singularity');
    legend('\sigma_1', '\sigma_2', '\sigma_3', '\sigma_4', '\sigma_5', '\sigma_6');
    grid on;
end

%% Save results
fprintf('\nSaving results...\n');
save('franka_singularities.mat', 'singularities', 'singularity_types', 'J_7_func');
fprintf('Results saved to franka_singularities.mat\n');

%% Helper function to classify singularity type
function type = classify_singularity(q, Q_min, Q_max)
    type = '';
    tol = 0.01;
    
    % Check if any joint is at its limit
    at_min = abs(q - Q_min) < tol;
    at_max = abs(q - Q_max) < tol;
    
    if at_min(4) || at_max(4)
        type = 'Elbow';
    elseif abs(q(5)) < 0.1
        type = 'Wrist';
    elseif any(at_min(1:3)) || any(at_max(1:3))
        type = 'Shoulder';
    elseif any(at_min) || any(at_max)
        type = 'Joint limit';
    else
        type = 'Interior';
    end
end

