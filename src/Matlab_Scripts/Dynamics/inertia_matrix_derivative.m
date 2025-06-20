function M_dot = inertia_matrix_derivative(M, q, dq)
    % Calculate the time derivative of the inertia matrix M
    %
    % Inputs:
    %   - M: Symbolic inertia matrix
    %   - q: Joint position vector [q1; q2; ...]
    %   - dq: Joint velocity vector [q1_dot; q2_dot; ...]
    %   - dq_dt: Joint acceleration vector (derivative of dq)
    %            Usually this is just dq for calculating S
    %
    % Output:
    %   - M_dot: Time derivative of M
    
    % Get the number of joints
    n = length(q);
    
    % Initialize M_dot with the same size as M
    M_dot = sym(zeros(size(M)));
    
    % Calculate the time derivative using the chain rule
    % dM/dt = sum_k (∂M/∂q_k) * dq_k
    for i = 1:n
        for j = 1:n
            M_dot_element = 0;
            
            % Sum over all joint variables
            for k = 1:n
                % Calculate partial derivative of M(i,j) with respect to q(k)
                partial_M = diff(M(i,j), q(k));
                
                % Multiply by the derivative of q(k) with respect to time
                M_dot_element = M_dot_element + partial_M * dq(k);
            end
            
            M_dot(i,j) = M_dot_element;
        end
    end
    
    % Simplify the result
    M_dot = simplify(M_dot);
end