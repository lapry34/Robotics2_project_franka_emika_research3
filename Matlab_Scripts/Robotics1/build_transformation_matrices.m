function A = build_transformation_matrices(DHTABLE)
    %% Build transformation matrices for each link
    N = size(DHTABLE, 1);
    A = cell(1, N);

    for i = 1:N
        alpha = DHTABLE(i, 1);
        a = DHTABLE(i, 2);
        d = DHTABLE(i, 3);
        theta = DHTABLE(i, 4);

        TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                  0             sin(alpha)             cos(alpha)            d;
                  0               0                      0                   1];

        A{i} = TDH;
    end
end

