function A = simplify_fatto_bene(A, rows, cols)
    for row=1:rows
        for col=1:cols
            A(row,col) = expand(A(row,col));
            A(row,col) = simplify(A(row,col));
        end
    end   
end