function [S] = factorization_S_from_inertia_matrix(M, q, dq, verbose)
    % Takes as inputs:
    %   - M = the symbolic inertia matrix
    %   - q = a vertical vector of joint positions
    %   - dq = a vertical vector of joint velocities
    % Output:
    %   - S = the factorization matrix such that S(q, dq)*dq = c(q, dq)
    
    % Get the number of joints
    n = length(q);

    if nargin < 4
        verbose = false;
    end
    
    % Calculate M_dot (the time derivative of M)
    M_dot = inertia_matrix_derivative(M, q, dq);

    if verbose
        disp('M_dot:');
        disp(M_dot);
    end
    
    % Initialize the Coriolis matrix S
    %S = sym(zeros(n, n));
    S = sym([]);

    cell_c_k = christoffel_symbols(M, q);

    for i = 1:n
        s_i = dq' * cell_c_k{i};
        S = [S; s_i];
    end
    
    % Calculate S using Christoffel symbols
    %for i = 1:n
    %    for j = 1:n
    %        for k = 1:n
    %            % Contribution to S from Christoffel symbols of the first kind
    %            S(i,j) = S(i,j) + 0.5 * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i))) * dq(k);
    %        end
    %    end
    %end
    
    % Simplify the result
    S = simplify(S);
    
    % Optional: Verify that S*dq equals c 
    % We can use the inertia_matrix_to_coriolis function to compute c
    try
        [c, ~] = inertia_matrix_to_coriolis(M, q, dq);
        S_dq = S * dq;
        c_verification = simplify(S_dq - c);
        if verbose
            disp('Verification (S*dq - c):');
            disp(c_verification);
        end
    catch
        if verbose
            disp('inertia_matrix_to_coriolis function not available for verification');
        end
    end
    
    % Check if M_dot - 2*S is skew-symmetric
    verification = simplify(M_dot - 2*S);
    skew_check = simplify(verification + transpose(verification));
    
    % Display the results
    if verbose
        disp('factorization of Coriolis matrix C by S:');
        disp(S);
        disp('M_dot - 2*S:');
        disp(verification);
        disp('Check if M_dot - 2*S is skew-symmetric (should be zeros):');
        disp(skew_check);
    end
end