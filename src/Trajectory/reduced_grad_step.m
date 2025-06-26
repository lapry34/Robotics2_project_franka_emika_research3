function dq = reduced_grad_step(q, dr, qA_idx, qB_idx, p_d, Kp)
    % J is the Jacobian of the constraint function r(q)
    % dr is the change in r, which is a vector
    % dt is the time step for the update
    % qA_idx: indices of the elements of q that are in A
    % qB_idx: indices of the elements of q that are in B
    
    % check if dr has orientation angles
    if length(dr) == 6
        orientation = true;
    else
        orientation = false;
    end

    if nargin < 3  
        p_d = get_p(q, orientation); % default to current end-effector position
    end

    J = get_J(q, orientation);   % <- numerica  (3x7 o 6x7)

    [M, N] = size(J); % M = 3 or 6, N = 7 (number of joints)
    Id = eye(N-M); % identity matrix of size (M-N)x(M-N)

    disp("J: ");
    disp(J);

    J_a = J(:, qA_idx); % (3x7 o 6x7) -> (3xN_a o 6xN_a) where N_a = length(qA_idx)
    J_b = J(:, qB_idx); % (3x7 o 6x7) -> (3xN_b o 6xN_b) where N_b = length(qB_idx)

    disp("J_a: ");
    disp(J_a);
    disp("J_b: ");
    disp(J_b);

    J_a_inv = pinv(J_a); % (N_a x M) matrix, where N_a = length(qA_idx), M = 3 or 6
    %J_a_inv = DLS(J_a); % damped least squares inverse of J_a

    % H_man = sqrt(det(J * J')); % maximize distance from singularities  (6x7 * 7x6 = 6x6)
    H_man = @(q) sqrt(det(get_J(q) * get_J(q)'));
    %H_range = @(q) H_range_dist(q);

    H = H_man;

    grad_H = num_diff(H, q)'; % numerical gradient of H (transposed Jacobian of scalar function)

    F = [-(J_a_inv * J_b)', Id]; % (N x N) matrix for reduced gradient step
    grad_H_b_prime = F * grad_H; % (N x 1) gradient of H with respect to qB modified for RG.


    p = get_p(q, orientation); % end-effector position
    e = p_d - p; % error vector

    dq_b = grad_H_b_prime;
    dq_a = J_a_inv * (dr -J_b*dq_b + Kp*e); % joint velocities for A (N_a x 1)

    dq = zeros(N, 1); % initialize full joint velocity vector
    dq(qA_idx) = dq_a; % assign joint velocities for A
    dq(qB_idx) = dq_b; % assign joint velocities for B

    %dq_full = pinv(J) * (dr + Kp * e);
end 





