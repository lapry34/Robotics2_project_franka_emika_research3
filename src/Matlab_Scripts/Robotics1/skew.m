function S = skew(x,y,z)
    S = [0, -z, y;
         z, 0, -x;
         -y, x, 0];
end
