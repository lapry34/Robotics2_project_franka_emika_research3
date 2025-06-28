function ddq = reduced_grad_step_acc(q, dq, ddr, qA_idx, qB_idx, p_d, dp_d, alpha, damp, Kp, Kd)
    % J is the Jacobian of the constraint function r(q)
    % dr is the change in r, which is a vector
    % dt is the time step for the update
    % qA_idx: indices of the elements of q that are in A
    % qB_idx: indices of the elements of q that are in B
    
    % check if ddr has orientation angles
    if length(ddr) == 6
        orientation = true;
    else
        orientation = false;
    end

    J = get_J(q, orientation);   % <- numerica  (3x7 o 6x7)
    J_dot = get_J_dot(q, dq, orientation); % numerical Jacobian time derivative

    [M, N] = size(J); % M = 3 or 6, N = 7 (number of joints)
    Id = eye(N-M); % identity matrix of size (M-N)x(M-N)

    x_ddot = ddr - J_dot * dq; % acceleration adjusted with J_dot

%     disp("J: ");
%     disp(J);

    J_a = J(:, qA_idx); % (3x7 o 6x7) -> (3xN_a o 6xN_a) where N_a = length(qA_idx)
    J_b = J(:, qB_idx); % (3x7 o 6x7) -> (3xN_b o 6xN_b) where N_b = length(qB_idx)

    % disp("J_a: ");
    % disp(J_a);
    % disp("J_b: ");
    % disp(J_b);

    J_a_inv = inv(J_a); % (N_a x M) matrix, where N_a = length(qA_idx), M = 3 or 6
    %J_a_inv = DLS(J_a); % damped least squares inverse of J_a

    H = @(q) simpler_H(q); % function to maximize distance from singularities

    grad_H = num_diff(H, q)'; % numerical gradient of H (transposed Jacobian of scalar function)

    grad_H = grad_H - damp * dq; % damping term (can be adjusted)

    F = [-(J_a_inv * J_b)', Id]; % (N x N) matrix for reduced gradient step
    grad_H_b_prime = F * alpha * grad_H; % (N x 1) gradient of H with respect to qB modified for RG.

    if nargin < 6  
        PD_control = 0;
    else
        p = get_p(q, orientation); % end-effector position
        e = p_d - p; % error vector
        e_dot = dp_d - J * dq; % error derivative
        PD_control = Kp * e + Kd * e_dot; % PD control term
    end


    ddq_b = grad_H_b_prime;
    ddq_a = J_a_inv * (x_ddot -J_b*ddq_b + PD_control); % joint velocities for A (N_a x 1)

    ddq = zeros(N, 1); % initialize full joint velocity vector
    ddq(qA_idx) = ddq_a; % assign joint velocities for A
    ddq(qB_idx) = ddq_b; % assign joint velocities for B

end 





