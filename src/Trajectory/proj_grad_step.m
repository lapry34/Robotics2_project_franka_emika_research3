function dq = proj_grad_step(q, dr, p_d)
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

    if nargin < 3  
        p_d = get_p(q, orientation); % default to current end-effector position
    end

    J = get_J(q, orientation);   % <- numerica  (3x7 o 6x7)


    pinv_J = pinv(J);
    
    % H_man = sqrt(det(J * J')); % maximize distance from singularities  (6x7 * 7x6 = 6x6)
    H_man = @(q) sqrt(det(get_J(q, orientation) * get_J(q, orientation)'));
    %H_range = @(q) H_range_dist(q);

    H = H_man;

    grad_H = num_diff(H, q)'; % numerical gradient of H (transposed Jacobian of scalar function)
    
    %disp('Gradient of H:');
    %disp(grad_H);

    p = get_p(q, orientation); % end-effector position
    e = p_d - p; % error vector
    Kp = 3*eye(length(e)); % proportional gain matrix
    
    %dq = pinv_J * (dr + Kp * e) + (eye(length(q)) - pinv_J * J) * grad_H;
    dq = grad_H + pinv_J * (dr - J * grad_H + Kp*e); % faster version
end 