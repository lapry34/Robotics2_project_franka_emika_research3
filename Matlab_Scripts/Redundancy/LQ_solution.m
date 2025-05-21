function [q_star, lambda] = LQ_solution(q, q0, W, J, r)
    % function H(q) = 1/2 * (q - q0)' * W * (q - q0) 
    % subject to J * q = r
    % Lagrangian L = H(q) - lambda' * (J * q - r) =
    % H(q) + lambda'*(r - J*q)

    W_inv = inv(W);

    lambda = inv(J * W_inv * J') * (r - J * q0);
    q_star = q0 + W_inv * J' * lambda;
end