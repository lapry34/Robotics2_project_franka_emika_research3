function [A, B] = trajectory_linearization(M, c, G, q_d, q_d_dot, q_d_ddot)
    % Linearize the trajectory using the given inertia matrix, coriolis matrix, and gravity vector
    % M: Inertia matrix
    % c: Coriolis matrix
    % G: Gravity vector
    % q_d: Desired joint positions
    % q_d_dot: Desired joint velocities
    % q_d_ddot: Desired joint accelerations

    %all M c, G are in q_d

    % Get the number of joints
    n = length(q_d);
    
    % Calculate C1 (partial derivative of c with respect to q)
    C1 = jacobian(c, q_d);
    
    % Calculate C2 (partial derivative of c with respect to q_dot)
    C2 = jacobian(c, q_d_dot);
    
    % Calculate D matrix as shown in the derivation
    D = G;
    D = D + C1;
    
    % Add the summation term from the equation
    for i = 1:n
        % Create the ith unit vector (ith row of identity matrix)
        e_i = zeros(n, 1);
        e_i(i) = 1;
        
        % Calculate partial derivative of M with respect to q_i
        partial_M_i = jacobian(M, q_d(i));
        
        % Add to D matrix
        D = D + partial_M_i * q_d_ddot(i) * e_i';
    end
    
    % Calculate M inverse
    M_inv = inv(M);
    
    % Create zero and identity matrices of appropriate size
    Zero_n = zeros(n, n);
    I_n = eye(n);
    
    % Construct A matrix
    A = [Zero_n, I_n; 
         -M_inv*D, -M_inv*C2];
    
    % Construct B matrix
    B = [zeros(n, n); 
         M_inv];
end