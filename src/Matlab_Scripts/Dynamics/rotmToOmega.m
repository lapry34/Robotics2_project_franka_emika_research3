function [w_x, w_y, w_z] = rotmToOmega(R, q, dq)
    % Initialize R_dot with zeros of the same size as R
    R_dot = sym(zeros(size(R)));
    
    % Handle the case where q is a vector of symbolic variables
    for i = 1:length(q)
        R_dot = R_dot + diff(R, q(i)) * dq(i);
    end
    
    R_dot = simplify(R_dot);
    S_w = R_dot * R';
    w_x = S_w(3, 2);
    w_y = S_w(1, 3);
    w_z = S_w(2, 1);
    w_x = simplify(w_x);
    w_y = simplify(w_y);
    w_z = simplify(w_z);
end
