function [c, C] = inertia_matrix_to_coriolis(M, q, dq, verbose)
    % Takes as inputs:
    %   - M = the inertia matrix
    %   - q = a vertical vector of q values
    %   - dq = a vertical vector dot_q
    % Output:
    %   - c = robot centrifugal and Coriolis term
    %   - C = Christoffel matrices

    % Initialize variables
    n = length(q); % Number of joints
    C = cell(n, 1); % Initialize cell array to store Christoffel matrices
    
    if nargin < 4
        verbose = false;
    end

    % Loop over each joint
    for i = 1:n
        % Compute Christoffel matrix for the i-th joint
        if verbose
            disp(['Christoffel matrix for joint ', num2str(i)])
        end
        Mi = M(:, i); % Select the i-th column of the inertia matrix
        Ci = (1/2) * (jacobian(Mi, q) + jacobian(Mi, q)' - diff(M, q(i)));
        C{i} = Ci; % Store the Christoffel matrix
        
        % Display the Christoffel matrix
        if verbose
            disp(['C', num2str(i), ' = ']);
            disp(Ci);
        end
    end
    
    % Compute robot centrifugal and Coriolis terms
    if verbose
        disp("Robot centrifugal and Coriolis terms")
    end
    c = sym(zeros(n, 1));
    for i = 1:n
        c(i) = dq' * C{i} * dq;
        if verbose
            disp(['c', num2str(i), ' = ']);
            disp(c(i));
        end
    end
end