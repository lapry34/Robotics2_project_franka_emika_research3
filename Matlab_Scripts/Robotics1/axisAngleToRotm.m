
function R = axisAngleToRotm(r, theta)

    r = r(:);

    S = [0, -r(3), r(2);
         r(3), 0, -r(1);
         -r(2), r(1), 0];

    c_t = cos(theta);
    s_t = sin(theta);

    rrT = r * r.';

    R = rrT + (eye(3) - rrT) * c_t + S * s_t;
end