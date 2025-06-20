function omega = RdotToOmega(Rdot, R)
        s_w = Rdot * R';
        omega = [s_w(3,2); s_w(1,3); s_w(2,1)];
        omega = simplify(omega);
end   
