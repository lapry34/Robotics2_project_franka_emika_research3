clear variables;
clc

syms l1 l2 q1 q2 q3 q4 real

% give him the joints to computer analytical jacobian
joints = [q1, q2, q3, q4];


position = [
    q2*cos(q1) + q4*cos(q1 + q3);
    q2*sin(q1) + q4*sin(q1 + q3);
    q1 + q3
    ];

% Compute the analytical Jacobian
J = jacobian(position, joints);
disp('Analytical Jacobian matrix:')
disp(J)

% compute determinant of the jacobian
detJ = simplify(det(J*J'));

disp('Determinant of the Jacobian:')
disp(detJ)

% compute the singularities
sing_1 = subs(J, [q2, q3], [0, 0]);
sing_2 = subs(J, [q2, q3], [0, pi]);
disp('Singularity 1 (q2 = 0, q3 = 0):')
disp(sing_1)
disp('Singularity 2 (q2 = 0, q3 = pi):')
disp(sing_2)

% null space of the jacobian
nullJ = null(J);
nullJ = simplify(nullJ);
disp('Null space of the Jacobian:')
disp(nullJ)

% null space of the transpose of the jacobian
nullJT = null(J');
nullJT = simplify(nullJT);
disp('Null space of the transpose of the Jacobian:')
disp(nullJT)