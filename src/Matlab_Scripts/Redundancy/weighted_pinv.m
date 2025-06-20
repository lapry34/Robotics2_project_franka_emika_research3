function Jw_pinv = weighted_pinv(J, W)
    
    W_inv = inv(W);

    %check if rank is full
    if rank(J) == size(J, 2)
        Jw_pinv = simplify(W_inv * J' * inv(J * W_inv * J'));
        return
    end
    W_neghalf = W^(-1/2);
    Jw_pinv = W_neghalf * pinv(J * W_neghalf);
    
end


