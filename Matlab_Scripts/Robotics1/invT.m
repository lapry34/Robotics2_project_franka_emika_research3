function inverseT = invT(T)
    %% Compute the inverse of a transformation matrix
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    inverseT = [R' -R'*p; 0 0 0 1];
end
