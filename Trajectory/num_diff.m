% Define function to compute the numerical Jacobian
function grad = num_diff(f, x, delta)

    if nargin < 3
        delta = 1e-6;  % default step size for numerical differentiation
    end

    n = numel(x);
    fx = f(x);
    m = numel(fx);
    grad = zeros(m, n);
    for i = 1:n
        x_forward = x;
        x_backward = x;
        x_forward(i) = x_forward(i) + delta;
        x_backward(i) = x_backward(i) - delta;

        grad(:, i) = (f(x_forward) - f(x_backward)) / (2 * delta);  % central difference
    end
end