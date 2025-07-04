function q_i = num_IK(r, q_est, verbose)

    if nargin < 3
        verbose = false;  % Default to not verbose
    end

    if nargin < 2
        q_est = zeros(7, 1);  % Default initial guess for joint angles
    end

    % Gradient Descent parameters
    alpha = 0.3;  % Learning rate for gradient descent
    tol = 1e-4;    % Tolerance for convergence
    max_iter = 30;
    found = false;

    q_i = q_est;  % Initial guess for joint angles

    if length(r) == 3
        orientation = false;  % If only position is given, do not consider orientation
    else
        orientation = true;  % If orientation is included, set flag to true
    end

    % Newton's Method
    for j = 1:max_iter
        
        error = r - get_p(q_i, orientation);  % Compute the error between desired position and current position
        if norm(error) < tol
            if verbose
                fprintf('Newton Method converged in %d iterations\n', j);
            end
            found = true;
            break;
        end

        J_curr = get_J(q_i, orientation);  % Compute the Jacobian at the current joint angles

        min_singular = svds(J_curr, 1, 'smallest');

        if min_singular > 1e-8 && min_singular < 1e-4 
            
            q_i = q_i + alpha * J_curr' * error;  % Gradient descent step (Armijo can be added here)
            
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