clc
clear
digits 4

% addpath("/Matlab_Scripts/Dynamics/") %no dynamics for now...
addpath("../Matlab_Scripts/Redundancy/")
addpath("../Matlab_Scripts/Robotics1/")

% From https://frankaemika.github.io/docs
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]';
a_i = [0; 0; 0; a4; a5; 0; a7; 0];
d_i = [d1; 0; d3; 0; d5; 0; 0; d_e];
alpha_i = [0; -pi/2; pi/2; pi/2; -pi/2; pi/2; pi/2; 0];
% a_i = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
% d_i = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
theta_i = [theta; 0];

DH_table = [alpha_i, a_i, d_i, theta_i]
T_ee = DHMatrix(DH_table);
p_ee = affine_get_translation(T_ee);
R_0_7 = affine_get_R(T_ee);

% geometric jacobian expressed in RF_0
% joints_str = 'RRRRRRR';
% [Jl, Ja] = geometric_jacobian(p_ee, joints_str, theta, DH_table);
% J_geom = [Jl; Ja];
% J_geom = simplify(J_geom);

% geometric jacobian expressed in RF_0
N=8;
joints_str = 'RRRRRRR-';
J_geom = geometric_jacobian_deluca(DH_table, joints_str, N);

% geometric jacobian expressed in RF_i
i = 7;
T0i = DHMatrix(DH_table(1:i,:));
R_0_i = affine_get_R(T0i);
R_i_0 = R_0_i';

R_j = blkdiag(R_i_0, R_i_0);

J_e_r = R_j * J_geom;
J_e_r = simplify_fatto_bene(J_e_r);

J_e_subset = J_e_r(:, 5:7)
ind_rows = [4, 5, 6];
dep_rows = [1, 2, 3];

R_ind = J_e_subset(ind_rows, :);
R_dep = J_e_subset(dep_rows, :);

% Solve R_ind' * x = R_dep(1,:)'
coeff_1 = R_ind' \ R_dep(1,:)';   
coeff_2 = R_ind' \ R_dep(2,:)';
coeff_3 = R_ind' \ R_dep(3,:)';

coefficients = [coeff_1'; coeff_2'; coeff_3'];


%coefficients = [sin(theta6) * coeff_1'; coeff_2'; sin(theta6) * coeff_3'];

fprintf('Row %d is a combination of the rows 4, 5, 6 with coeff:', dep_rows(1));
disp(simplify(coeff_1'));

fprintf('Row %d is a combination of the rows 4, 5, 6 with coeff:', dep_rows(2));
disp(simplify(coeff_2'));

fprintf('Row %d is a combination of the rows 4, 5, 6 with coeff:', dep_rows(3));
disp(simplify(coeff_3'));


Coeff1 and Coeff3 have a sin(theta6) division that we can omit by multiplying all the row for that amount
% The t is added to denote transformed matrix
J_e_t = J_e_r;
R4 = J_e_t(4, :); % 4th row of J_e_t
R5 = J_e_t(5, :); % 5th row of J_e_t
R6 = J_e_t(6, :); % 6th row of J_e_t
for i=dep_rows
     c4 = coefficients(i, 1);
     c5 = coefficients(i, 2);
     c6 = coefficients(i, 3);
     if i == 2
         J_e_t(i, :) = J_e_t(i, :) - c4 * R4 - c5 * R5 - c6 * R6;
     else
         s6 = sin(theta6);
         J_e_t(i, :) = s6 * J_e_t(i, :) - s6 * c4 * R4 - s6 *c5 * R5 - s6 *c6 * R6;
     end
end

J_e_t = simplify_fatto_bene(J_e_t)

J_e_t_11 = J_e_t(1:3, 1:4)
J_e_t_21 = J_e_t(4:6, 1:4);
J_e_t_22 = J_e_t(4:6, 5:7)

rank(J_e_t_11)
equations1 = analysis_singularities(J_e_t_11, false)
solve(equations1(3), theta)

rank(J_e_t_22)
equations2 = analysis_singularities(J_e_t_22, false)

J_s_22 = subs(J_e_t_22, theta6, 0)
rank(J_s_22)

% % https://frankaemika.github.io/docs/control_parameters.html
% % limits-for-franka-research-3
% Q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]'; % [rad]
% Q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]'; % [rad]
% Q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
% Q_ddot_max = 10 * ones(1, N); % [rad/s^2]
% Q_dddot_max = 5000 * ones(1, N);

% H_range = compute_H_range(theta, Q_min, Q_max);
% q0 = - jacobian(H_range, theta)';