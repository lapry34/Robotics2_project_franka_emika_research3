function model = to_motor_side(M, C, g, Fv, I_motors, n_r, ddtheta_m, ddq, dq)
     
    % Extract diagonal matrix (keep matrix form)
    M_diag_matrix = diag(diag(M));  % First diag() extracts vector, second creates matrix

    % Extract the matrix without the diagonal M_barred
    M_barred = M - M_diag_matrix;  % Now subtracting a matrix from matrix

    model = (I_motors + diag(M_diag_matrix)/ n_r^2) * ddtheta_m + M_barred * ddq + C * dq + g + Fv;
    model = simplify(model);
end
