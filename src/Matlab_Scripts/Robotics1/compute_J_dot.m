function J_dot = compute_J_dot(J, q, dq)
    % Compute the symbolic Jacobian derivative

    % Initialize the symbolic Jacobian derivative
    J_dot = sym(zeros(size(J)));

    % Iterate over each joint
    for i = 1:size(J, 2)
        % Compute the derivative of the Jacobian for the current joint
        J_dot(:, i) = diff(J(:, i), q(i)) * dq(i);
    end

    simplify(J_dot)
end