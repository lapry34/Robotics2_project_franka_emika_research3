function q_ddot = projected_gradient_dynamics(J, J_dot, r_ddot, q_dot, grad_H, K)
    % Inputs:
    %   J : (m x n) jacobian matrix of the task 
    %   J_dot : (m x n) time derivative of the jacobian matrix
    %   r_ddot : (m x 1) desired acceleration of the task
    %   q_dot : (n x 1) velocity of the joints
    %   grad_H : (n x 1) gradient of the task function wrt q
    %   K : (n x n) positive definite matrix for the damping

    % Output:
    %   q_ddot : (n x 1) acceleration of the joints


    x_ddot = r_ddot - J_dot * q_dot;
    J_inv = pinv(J);
    q0_dot = grad_H - K * q_dot;

    %q_ddot = q0_dot + pinv(J) * (x_ddot - J * q0_dot);
    
    q_ddot = J_inv * x_ddot + (eye(length(J)) - J_inv * J) * q0_dot;
    q_ddot = simplify(q_ddot);
end