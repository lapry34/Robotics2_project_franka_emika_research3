function J_dot = get_J_dot(q, dq,orientation, delta)
%GET_J_DOT Numerically computes the time derivative of the Jacobian matrix J(q).
%
%   J_dot = GET_J_DOT(q, dq) returns the matrix time derivative J_dot given
%   the joint positions q and joint velocities dq. The Jacobian J(q) must be
%   provided by a function handle GET_J that takes q and returns the Jacobian.
%
%   J_dot = GET_J_DOT(q, dq, delta) allows specification of the finite
%   difference step size DELTA (default 1e-6).
%
%   The method uses central finite differences:
%       dJ/dq_k \approx [J(q + \delta e_k) - J(q - \delta e_k)] / (2\delta)
%   and then applies the chain rule:
%       J\dot = \sum_k (dJ/dq_k) * dq_k
%

    if nargin < 4 || isempty(delta)
        delta = 1e-6; % default finite difference step size
    end
    if nargin < 3 || isempty(orientation)
        orientation = false; % default to not considering orientation
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
