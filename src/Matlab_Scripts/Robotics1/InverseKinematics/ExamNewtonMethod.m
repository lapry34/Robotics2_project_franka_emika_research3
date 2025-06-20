clear variables;
clc;


p = @(q) [q(3)*cos(q(2)); q(3)*sin(q(2)); q(1)]; %Cylindrical robot


J = @(q) [
    0, -q(3)*sin(q(2)), cos(q(2)); 
    0, q(3)*cos(q(2)), sin(q(2)); 
    1, 0, 0]; %Jacobian of p(q)


% Target value for which we want to find the inverse
p_d = [1; -1; 3];  

% Initial guess for x
q_A = [-2; 0.7*pi; sqrt(2)];
q_B = [2; pi/4; sqrt(2)];
q_i = q_B;

% Gradient Descent parameters
alpha = 1;  % Learning rate for gradient descent
tol = 1e-4;    % Tolerance for convergence
max_iter = 9;

% Newton's Method
for j = 1:max_iter
    error = p_d - p(q_i);
    if norm(error) < tol
        fprintf('Newton Method converged in %d iterations\n', j);
        break;
    end

    Jac = J(q_i);


    %q_i = q_i - alpha * (J(q_i) \ error);  % Newton's Method update using the Jacobian inverse
    q_i = q_i + alpha * (pinv(Jac) * error);
end

% If Newton's Method did not converge, display a warning
if j == max_iter
    warning('Newton Method did not converge within the maximum iterations');
end


%
%disp("Iteration: ");
%disp(j);
%disp("Jacobian: ");
%disp(Jac);
%disp("Jacobian Determinant: ");
%disp(det(Jac));
%disp("p_i: ");
%disp(p(q_i));
%disp("q_i: ");
%disp(q_i);
%disp("Error: (p(q_i) - p_d) ");
%disp(error);