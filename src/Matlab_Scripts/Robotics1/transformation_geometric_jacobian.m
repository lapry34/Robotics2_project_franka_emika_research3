function J = transformation_geometric_jacobian(J, R, r)
    %% Compute the geometric Jacobian for a transformation matrix
    M1 = [R, zeros(3); zeros(3), R];
    M2 = [eye(3), -skew(r); zeros(3), eye(3)];
    J = M1 * M2 * J;
end
