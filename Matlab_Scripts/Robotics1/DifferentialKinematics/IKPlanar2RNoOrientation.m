clc;
clear all;

% Define symbolic variables

syms q1 q2 px py real
l1 = 1;
l2 = 0.5;

% Define the position of the end effector

pos = [
      l1*cos(q1) + l2*cos(q1 + q2); 
      l1*sin(q1) + l2*sin(q1 + q2)
      ];

% inverse kinematics
% solve for q1 and q2

c2 = (px^2 + py^2 - l1^2 - l2^2)/(2*l1*l2);
s2_1 = sqrt(1 - c2^2);
s2_2 = -sqrt(1 - c2^2);

q2_1 = atan2(s2_1, c2);
q2_2 = atan2(s2_2, c2);


detq1 = l1^2 + l2^2 + 2*l1*l2*c2;
s1_1 = ((l1 + l2*c2)*py - l2*s2_1*px)/detq1;
s1_2 = ((l1 + l2*c2)*py - l2*s2_2*px)/detq1;
c1_1 = ((l1 + l2*c2)*px + l2*s2_1*py)/detq1;
c1_2 = ((l1 + l2*c2)*px + l2*s2_2*py)/detq1;

q1_1 = atan2(s1_1, c1_1);
q1_2 = atan2(s1_2, c1_2);

p0 = [0; 1];

q1_1 = vpa(subs(q1_1, [px; py], p0));
q2_1 = vpa(subs(q2_1, [px; py], p0));

q1_2 = vpa(subs(q1_2, [px; py], p0));
q2_2 = vpa(subs(q2_2, [px; py], p0));

% Display the results
disp("First solution");
fprintf('q1 = %f\n', q1_1);
fprintf('q2 = %f\n', q2_1);

disp("Second solution");
fprintf('q1 = %f\n', q1_2);
fprintf('q2 = %f\n', q2_2);