function q_dot = projected_gradient(J, r_dot, q0_dot) %q0_dot is grad H(q) wrt q
   
    I = eye(length(J));
    %q_dot = pinv(J) * r_dot + (I - pinv(J) * J) * q0_dot; %original version
    q_dot = q0_dot + pinv(J) * (r_dot - J * q0_dot); %faster version
    q_dot = simplify(q_dot);
end