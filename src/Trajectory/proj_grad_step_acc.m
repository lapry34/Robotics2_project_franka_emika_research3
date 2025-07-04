function ddq = proj_grad_step_acc(q, dq, ddr, p_d, dp_d, Kp, Kd)
    % J is the Jacobian of the constraint function r(q)
    % dr is the change in r, which is a vector
    % dt is the time step for the update

    % q = q_old + dt * dq_old;
    
    % check if dr has orientation angles
    if length(ddr) == 6
        orientation = true;
    else
        orientation = false;
    end

    J = get_J(q, orientation);   % <- numerica  (3x7 o 6x7)
    J_dot = get_J_dot(q, dq, orientation); % numerical Jacobian time derivative

    x_ddot = ddr - J_dot * dq; % acceleration adjusted with J_dot

    pinv_J = pinv(J);
    
    % H_man = sqrt(det(J * J')); % maximize distance from singularities  (6x7 * 7x6 = 6x6)
    H_man = @(q) sqrt(det(get_J(q, orientation) * get_J(q, orientation)'));
    %H_range = @(q) H_range_dist(q);

    H = H_man;

    grad_H = num_diff(H, q)'; % numerical gradient of H (transposed Jacobian of scalar function)
    
    damp = 2;
    q0_ddot = grad_H - damp * dq; % damping term (can be adjusted)


    if nargin < 4  
        PD_control = 0;
    else
        p = get_p(q, orientation); % end-effector position
        e = p_d - p; % error vector
        e_dot = dp_d - J * dq; % error derivative
        PD_control = Kp * e + Kd * e_dot; % PD control term
    end


    ddq = q0_ddot + pinv_J * (x_ddot - J * q0_ddot + PD_control); % faster version

end 





