function J_DLS = J_damped_least_squares(J, mu)
    
    size_J = size(J);
    M = size_J(1);
    J_DLS = J' * inv(J * J' + mu^2 * eye(M));
 
end