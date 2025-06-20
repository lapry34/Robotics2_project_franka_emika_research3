function J_new = changeFrameGeomJacobian(J, R)
% changeFrameGeomJacobian - changes the frame of a geometric Jacobian
% J - geometric Jacobian (from 0 to N)
% R - rotation matrix from 0 to i

    R_big = zeros(6);
    R_big(1:3, 1:3) = R;
    R_big(4:6, 4:6) = R; % R_big is a 6x6 matrix with R in the upper left corner and lower right corner

    J_new = R_big * J;
end