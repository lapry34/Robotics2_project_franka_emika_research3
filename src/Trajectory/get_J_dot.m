function J_dot = get_J_dot(q, dq, orientation, delta)
%GET_J_DOT Numerically computes the time derivative of the Jacobian matrix J(q).
%
% ...existing code...

    if nargin < 4
        delta = 1e-6; % Default perturbation for numerical differentiation
    end
    if nargin < 3
        orientation = false; % Default to not considering orientation
    end

    % Ensure q is a column vector
    if isrow(q)
        q = q(:);
    end
    % Ensure dq is a column vector
    if isrow(dq)
        dq = dq(:);
    end

    % Current Jacobian and its size
    J       = get_J(q, orientation);
    [n, m]  = size(J);

    % Pre‑allocate J_dot
    J_dot = zeros(n, m);

    % Loop over each joint coordinate
    for k = 1:m
        q_fwd      = q; q_fwd(k) = q_fwd(k) + delta;
        q_bwd      = q; q_bwd(k) = q_bwd(k) - delta;

        J_fwd      = get_J(q_fwd, orientation);
        J_bwd      = get_J(q_bwd, orientation);

        dJ_dqk     = (J_fwd - J_bwd) / (2 * delta); % ∂J/∂q_k

        % Chain‑rule contribution: (∂J/∂q_k) * dq_k
        J_dot      = J_dot + dJ_dqk * dq(k);
    end
end