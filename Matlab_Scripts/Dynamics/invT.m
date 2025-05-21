function T_inv = invT(T)
    T_inv = zeros(4,4);

    T_inv(1:3,1:3) = T(1:3,1:3)';
    T_inv(1:3,4) = -T(1:3,1:3)'*T(1:3,4);
    T_inv(4,4) = 1;
end