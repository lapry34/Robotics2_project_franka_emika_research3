clear variables;
clc

syms l1 l2 l3 q1(t) q2(t) q3(t) q1_dot(t) q2_dot(t) q3_dot(t) q1_ddot(t) q2_ddot(t) q3_ddot(t) t 

% give him the joints to computer analytical jacobian
joints = [q1, q2, q3];


% plana 3R robot

position = [
    l1*cos(q1) + l2*cos(q1 + q2) + l3*cos(q1 + q2 + q3);
    l1*sin(q1) + l2*sin(q1 + q2) + l3*sin(q1 + q2 + q3);
    q1 + q2 + q3;
];


J = jacobian(position, joints);

J_dot = diff(J, t);
J_dot = subs(J_dot, diff(q1(t), t), q1_dot);
J_dot = subs(J_dot, diff(q2(t), t), q2_dot);
J_dot = subs(J_dot, diff(q3(t), t), q3_dot);

J_ddot = diff(J_dot, t);
J_ddot = subs(J_ddot, diff(q1(t), t), q1_dot);
J_ddot = subs(J_ddot, diff(q2(t), t), q2_dot);
J_ddot = subs(J_ddot, diff(q3(t), t), q3_dot);
J_ddot = subs(J_ddot, diff(q1_dot(t), t), q1_ddot);
J_ddot = subs(J_ddot, diff(q2_dot(t), t), q2_ddot);
J_ddot = subs(J_ddot, diff(q3_dot(t), t), q3_ddot);

disp("J: ")
disp(J);

disp("J_dot: ")
disp(J_dot)

disp("J_ddot: ")
disp(J_ddot)
