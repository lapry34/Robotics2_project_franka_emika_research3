function [M_r, n_r, u_r] = redundant_task_coordinate_transformation(M_q, n_q, u_q, q_dot, J, J_dot)

    J_inv = inv(J);
    M_q_inv = inv(M_q);

    M_r = J_inv' * M_q * J_inv;
    n_r = M_r *(J * M_q_inv * n_q - J_dot * q_dot);
    u_r = 0 %TODO
end