function tau = minimum_inverse_inertia_weighted_torque_norm_solution(J,  J_dot, q_dot, M, r_ddot, n, tau_0)
    %Inputs:
    %   J : (m x n) jacobian matrix of the task 
    %   J_dot : (m x n) time derivative of the jacobian matrix
    %   q_dot : (m x 1) velocity of the joints
    %   M : (n x n) inertia matrix
    %   n :  (n x 1) Coriolis and centrifugal forces, gravity vector, and other external forces
    %   tau_0 : (n x 1) nullspace 
    %Output:
    %   tau : (n x 1) minimum (inverse inertia weighted) torque norm solution for dynamic redundancy
    
    x_ddot = r_ddot - J_dot * q_dot;
    M_inv = inv(M);

    tau = J' * inv(J * M_inv * J') * (x_ddot + J * M_inv * n);

    if nargin > 6
        J_inv = pinv(J);
        num_rows = size(J_inv, 1);
        I = eye(num_rows);
        H = (inv(J * M_inv * J') * J * M_inv); %generalized inverse
        tau = tau + M * (I - J' * H) * tau_0;
    end

    tau = simplify(tau);

end