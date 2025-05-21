function [J, J_partial] = PartialGeometricJacobian(DHTABLE, joints)
    % Validate input
    assert(size(DHTABLE, 2) == 4, "DHTABLE must have 4 columns");
    assert(size(DHTABLE, 1) == length(joints), "DHTABLE must have the same number of rows as joints");
    % DHTable is alpha a d theta

    % Compute the transformation matrices
    A = build_transformation_matrices(DHTABLE);
    
    % Perform direct kinematics
    [p_vec, z_vec, T0N] = direct_kinematics(A);
    
    % Compute full geometric Jacobian
    J = compute_geometric_jacobian(p_vec, z_vec, joints);
    
    % Compute partial Jacobians
    J_partial = compute_partial_jacobians(p_vec, z_vec, joints);

    simplify(J);
end

function [p_vec, z_vec, T0N] = direct_kinematics(A)
    % Perform direct kinematics
    T = eye(4);
    N = length(A);

    p_i = T(1:3, 4);
    z_i = T(1:3, 3);

    p_vec = [p_i];
    z_vec = [z_i];

    for i = 1:N
        T = T * A{i};
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);

        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end

    T0N = T;
end

function J = compute_geometric_jacobian(p_vec, z_vec, joints_str)
    % Compute the full geometric Jacobian
    JP = [];
    JO = [];

    N = length(joints_str);

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
end

function J_partial = compute_partial_jacobians(p_vec, z_vec, joints_str)
    % Compute partial Jacobians from 0 to each joint
    N = length(joints_str);
    J_partial = cell(1, N);

    for k = 1:N
        % For each partial Jacobian up to joint k
        JP_k = [];
        JO_k = [];

        for i = 1:k
            p_i = p_vec(:, i);
            z_i = z_vec(:, i);
            
            if joints_str(i) == 'R'
                JP_k = [JP_k, cross(z_i, p_vec(:, k+1) - p_i)];
                JO_k = [JO_k, z_i];
            else
                JP_k = [JP_k, z_i];
                JO_k = [JO_k, [0; 0; 0]];
            end
        end

        % Pad with zeros to maintain consistent column count
        if size(JP_k, 2) < N
            JP_k = [JP_k, zeros(3, N - size(JP_k, 2))];
            JO_k = [JO_k, zeros(3, N - size(JO_k, 2))];
        end

        % Combine translational and rotational parts
        J_partial{k} = [JP_k; JO_k];
    end
end

function A = build_transformation_matrices(DHTABLE)
    % Build transformation matrices for each link
    N = size(DHTABLE, 1);
    A = cell(1, N);

    for i = 1:N
        alpha = DHTABLE(i, 1);
        a = DHTABLE(i, 2);
        d = DHTABLE(i, 3);
        theta = DHTABLE(i, 4);

        TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                  0             sin(alpha)             cos(alpha)            d;
                  0               0                      0                   1];

        A{i} = TDH;
    end
end

% Example usage:
% DHTABLE = [alpha a d theta];
% joints = 'RRR'; % Rotation joints
% [J, J_partial] = PartialGeometricJacobian(DHTABLE, joints);
% J_partial will be a cell array of partial Jacobians from 0 to each joint