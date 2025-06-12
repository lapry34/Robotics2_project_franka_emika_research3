function q_i = num_IK(r, verbose)

    if nargin < 2
        verbose = false;  % Default to not verbose
    end

    % Gradient Descent parameters
    alpha = 0.3;  % Learning rate for gradient descent
    tol = 1e-4;    % Tolerance for convergence
    max_iter = 9;
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

%        min_singular = svds(J_s, 1, 'smallest');
%        
%        if min_singular < 1e-4
%            fprintf('Jacobian is singular at iteration %d\n', j);
%            break;
%        end

        %q_i = q_i - alpha * (J(q_i) \ error);  % Newton's Method update using the Jacobian inverse
        q_i = q_i + alpha * (pinv(J_curr) * error);
    end

    if ~found && verbose
        fprintf('Newton Method did not converge\n');
    end

end