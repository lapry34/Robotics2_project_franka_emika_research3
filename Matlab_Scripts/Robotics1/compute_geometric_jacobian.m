function J = compute_geometric_jacobian(DHTable, joints_str)
    %% Compute the geometric Jacobian
    JP = [];
    JO = [];

    N = length(joints_str);

    A = build_transformation_matrices(DHTable, joints_str);
    [p_vec, z_vec, T0N] = direct_kinematics(A); 

    for i = 1:N
        p_i = p_vec(:, i);
        z_i = z_vec(:, i);
        if joints_str(i) == 'R'
            JP = [JP, cross(z_i, p_vec(:, end) - p_i)];
            JO = [JO, z_i];
        else
            JP = [JP, z_i];
            JO = [JO, [0; 0; 0]];
        end
    end

    J = [JP; JO];
    J = simplify(J);
    disp("Geometric Jacobian matrix:");
    disp(J);
end