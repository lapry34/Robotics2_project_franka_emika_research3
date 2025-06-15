function dq = proj_grad_step(q, dr)
    % J is the Jacobian of the constraint function r(q)
    % dr is the change in r, which is a vector
    % dt is the time step for the update

    % q = q_old + dt * dq_old;
    
    % check if dr has orientation angles
    if length(dr) == 6
        orientation = true;
    else
        orientation = false;
    end

    J = get_J(q, orientation);   % <- numerica

    pinv_J = pinv(J);
    
    % H_man = sqrt(det(J' * J)); % maximize distance from singularities
    grad_H = num_J(@(q) det(get_J(q)' * get_J(q)), q)'; % numerical gradient of H (transposed Jacobian of scalar function)
    
    dq = grad_H + pinv_J * (dr - J * grad_H); % faster version

end 





