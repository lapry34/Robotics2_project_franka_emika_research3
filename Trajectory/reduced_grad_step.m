function dq = reduced_grad_step(q, dr, p_d, qA_idx, qB_idx)
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

    J_a = J(:, qA_idx); % (3x7 o 6x7) -> (3xN_a o 6xN_a) where N_a = length(qA_idx)
    J_b = J(:, qB_idx); % (3x7 o 6x7) -> (3xN_b o 6xN_b) where N_b = length(qB_idx)


    id = eye(N - M); % identity matrix of size (N-M)x(N-M)
    J_a_inv = inv(J_a);

    J_inv = [J_a_inv; zeros(N - M, size(J_a_inv, 1))]; % (N x N_a)

    F = [-J_a_inv * J_b; id]; % (N x N) matrix
    FF = F * F'; % (N x N) matrix

    % H_man = sqrt(det(J * J')); % maximize distance from singularities  (6x7 * 7x6 = 6x6)
    H_man = @(q) sqrt(det(get_J(q) * get_J(q)'));
    %H_range = @(q) H_range_dist(q);

    H = H_man;

    grad_H = num_diff(H, q)'; % numerical gradient of H (transposed Jacobian of scalar function)
    
  

    p = get_p(q, orientation); % end-effector position
    e = p_d - p; % error vector
    Kp = 5*eye(length(e)); % proportional gain matrix

    dq = J_inv * (dr + Kp * e) + FF * grad_H; % compute joint velocity update
    % Reorder dq to match the original order of q
    dq_full = zeros(N, 1); % initialize full joint velocity vector

    for i = 1:length(qA_idx)
        dq_full(qA_idx(i)) = dq(i); % assign values for A joints
    end
    for i = 1:length(qB_idx)
        dq_full(qB_idx(i)) = dq(length(qA_idx) + i); % assign values for B joints
    end

    dq = dq_full; % return the full joint velocity vector
end 





