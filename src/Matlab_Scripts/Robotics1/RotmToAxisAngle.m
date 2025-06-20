
function [r_1, r_2, theta_1, theta_2] = RotmToAxisAngle(R)


    tolerance_s = 1e-6; 
    tolerance_theta = 1e-3;  

    R_12 = R(1,2);
    R_21 = R(2,1);

    R_13 = R(1,3);
    R_31 = R(3,1);

    R_23 = R(2,3);
    R_32 = R(3,2);

    R_11 = R(1,1);
    R_22 = R(2,2);
    R_33 = R(3,3);

    y = sqrt((R_12 - R_21)^2 + (R_13 - R_31)^2 + (R_23 - R_32)^2);
    x = R_11 + R_22 + R_33 - 1;

    theta_1 = atan2(y, x);

    s_1 = sin(theta_1);

    if abs(theta_1) < tolerance_theta
        error("Undefined! Theta approaches 0.");

    elseif (abs(s_1) < tolerance_s && abs(abs(theta_1) - pi) < tolerance_theta)
        r_1 = [sqrt(R_11 + 1); sqrt(R_22 + 1); sqrt(R_33 + 1)] / sqrt(2);
    else
        r_1 = [R_32 - R_23; R_13 - R_31; R_21 - R_12] / (2 * sin(theta_1));
    end
    if sign(r_1(1) * r_1(2)) ~= sign(R_12 / 2)
        r_1(1) = -r_1(1);
    end
    if sign(r_1(1) * r_1(3)) ~= sign(R_13 / 2)
        r_1(3) = -r_1(3);
    end
    if sign(r_1(2) * r_1(3)) ~= sign(R_23 / 2)
        r_1(2) = -r_1(2);
    end

    r_2 = -r_1;
    theta_2 = -theta_1;

end
