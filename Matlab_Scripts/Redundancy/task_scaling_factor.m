function [critical_joint, task_scaling_factor] = task_scaling_factor(J, x_dot, q_dot_0, Q_min_dot, Q_max_dot, verbose)

    if nargin < 6
        verbose = false;
    end

    [M, N] = size(J);
    q_dot_r = simplify(pinv(J) * x_dot);
    q_dot_n = simplify(proj_nullSpace(J) * q_dot_0);

    

    S_min = zeros(N, 1);
    S_max = zeros(N, 1);
    
    for i = 1:N
        s_min__ = (Q_min_dot(i)-q_dot_n(i)) / q_dot_r(i);
        s_max__ = (Q_max_dot(i)-q_dot_n(i)) / q_dot_r(i);
        
        if s_min__ > s_max__
            S_min(i) = s_max__;
            S_max(i) = s_min__;
        else
            S_min(i) = s_min__;
            S_max(i) = s_max__;
        end
    end
    

    [M, I] = min(S_max);
    s_max = M;
    s_min = max(S_min);
    
    critical_joint = I; 
    

    if s_min > s_max || s_max < 0 || s_min > 1
        task_scaling_factor = 0;
    else
        task_scaling_factor = s_max;
    end

    v_s = vpa(task_scaling_factor * x_dot);
    q_dot_scaled = vpa(task_scaling_factor * q_dot_r + q_dot_n);

    if verbose
        fprintf("Minimum norm solution: \n ")
        disp(q_dot_r)
        fprintf("Null space term: \n ")
        disp(q_dot_n)
        fprintf("\n The most critical joint is joint %d.\n", I)
        fprintf("The scaling factor is %.4f.\n", task_scaling_factor)
        fprintf("Therefore, the scaled task velocity is v_s = k * r_dot:")
        disp(v_s)
        fprintf("Therefore, the scaled joint velocity that recovers feasibility is q_dot_scaled = k * q_dot_r + q_dot_n:")
        disp(q_dot_scaled)
    end

    
end