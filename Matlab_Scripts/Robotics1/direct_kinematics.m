
function [p_vec, z_vec, T0N] = direct_kinematics(A)
    %% Perform direct kinematics
    T = eye(4);
    N = length(A);


    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    p_vec = [p_i];
    z_vec = [z_i];

    for i = 1:N
        T = T * A{i};
        T = simplify(T);
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);
        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end

    T0N = T;
end
