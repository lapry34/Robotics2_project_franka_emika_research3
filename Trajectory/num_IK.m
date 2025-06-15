function q_i = num_IK(r, verbose)

    if nargin < 2
        verbose = false;  % Default to not verbose
    end

    % Gradient Descent parameters
    alpha = 0.3;  % Learning rate for gradient descent
    tol = 1e-4;    % Tolerance for convergence
    max_iter = 30;
    found = false;

    q_i = zeros(7,1);  % Initial guess for joint angles

    % Newton's Method
    for j = 1:max_iter
        
        error = r - get_p(q_i);
        if norm(error) < tol
            if verbose
                fprintf('Newton Method converged in %d iterations\n', j);
            end
            found = true;
            break;
        end

        J_curr = get_J(q_i);  % Compute the Jacobian at the current joint angles
        J_curr = J_curr(1:3, :);  % Use only the first three rows for position control

        min_singular = svds(J_curr, 1, 'smallest');

        if min_singular > 1e-8 && min_singular < 1e-4 
            
            q_i = q_i + alpha * J_curr' * error;  % Gradient descent step
            
            if verbose
                fprintf('Using gradient descent update: min singular value = %f\n', min_singular);
            end

        else 
            %q_i = q_i - alpha * (J(q_i) \ error);  % Newton's Method update using the Jacobian inverse

            if verbose
                fprintf('Using pseudo-inverse update: min singular value = %f\n', min_singular);
            end

            J_pinv = pinv(J_curr);  % Use pseudo-inverse for the Jacobian
            q_i = q_i + alpha * J_pinv * error;
        end
    end

    if ~found && verbose
        fprintf('Newton Method did not converge\n');
    end

end