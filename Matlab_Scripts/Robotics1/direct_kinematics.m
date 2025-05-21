
function [p_vec, z_vec, T0N] = direct_kinematics(A)
    %% Perform direct kinematics
    T = eye(4);
    N = length(A);


    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    disp("p_0 = [" + join(string(p_i), "; ") + "];");
    disp("z_0 = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_i];
    z_vec = [z_i];

    for i = 1:N
        T = T * A{i};
        T = simplify(T);
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);
        disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
        disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end

    T0N = T;
    fprintf("\n\n__________________________________\n\n");
    disp("Final Transformation Matrix T0N:");
    disp(T0N);
end
