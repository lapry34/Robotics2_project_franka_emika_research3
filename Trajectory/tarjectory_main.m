clc
clear
digits 4

% addpath("/Matlab_Scripts/Dynamics/") %no dynamics for now...
addpath("./Matlab_Scripts/Redundancy/")
addpath("./Matlab_Scripts/Robotics1/")

% From https://frankaemika.github.io/docs
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

% a4 =0.0825; 
% a5 =-0.0825;
% a7 = 0.088;
% d3 = 0.316; d5 = 0.384; d_e = 0.107;
theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]';

a_i = [0; 0; 0; a4; a5; 0; a7; 0];
d_i = [d1; 0; d3; 0; d5; 0; 0; d_e]; 
alpha_i = [0; -pi/2; pi/2; pi/2; -pi/2; pi/2; pi/2; 0];

% a_i = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
% d_i = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
theta_i = [theta; 0];

N = 7;
Q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]'; % [rad]
Q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]'; % [rad]
Q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]'; % [rad/s]
Q_ddot_max = 10 * ones(1, N); % [rad/s^2]
Q_dddot_max = 5000 * ones(1, N);






