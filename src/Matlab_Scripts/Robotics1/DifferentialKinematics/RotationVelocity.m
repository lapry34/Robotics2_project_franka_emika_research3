clear all; clc;

syms alpha beta gamma real


disp("ciao");

angles = [pi/2, pi/4, pi/3];
R = anglesToRotm(angles, "ZYX", false);

disp("R:");
disp(R);

[a1, a2] = rotmToAngles(R, "ZYX", false);
[r1, r2, t1, t2] = RotmToAxisAngle(R);

theta_dot = pi/2; %rad/s 

omega = theta_dot * r1; % if axis angle representation is used

R_dot = rotmDot(R, omega);


disp("R_dot:");
disp(R_dot);

%try anglesDotToOmega
angles_dot = [0, 0, theta_dot];
omega = anglesDotToOmega(angles, angles_dot, "ZYX", false);

syms alpha beta gamma real 
syms alpha_dot beta_dot gamma_dot theta_dot real
angles = [alpha, beta, gamma];
angles_dot = [alpha_dot, beta_dot, gamma_dot];
omega = anglesDotToOmega(angles, angles_dot, "XYZ", false);


disp("omega:");
disp(omega);

T_xyz_e = anglesTimeTmatrix(angles, "XYZ", true);
disp("T_xyz_e:");
disp(T_xyz_e);


function T = anglesTimeTmatrix(angles, sequence, fixed)
    % angles: symbolic or numeric angles [alpha, beta, gamma]
    % sequence: rotation sequence (e.g., "ZYX")
    % fixed: boolean indicating fixed or rotating frame

    sequence = char(sequence);
    
    R1 = eye(3);
    R2 = Rot(angles(1), sequence(1));
    R3 = R2 * Rot(angles(2), sequence(2));

    n1 = numberAxis(sequence(1));
    n2 = numberAxis(sequence(2));
    n3 = numberAxis(sequence(3));

    if fixed 
        T = [R3(:,n3), R2(:,n2), R1(:,n1)];
    else 
        T = [R1(:,n1), R2(:,n2), R3(:,n3)];
    end

end

function n = numberAxis(axis)
    switch axis
        case 'X'
            n = 1;
        case 'Y'
            n = 2;
        case 'Z'
            n = 3;
        otherwise
            n = 0;
    end
end

function omega = anglesDotToOmega(angles, angles_dot, sequence, fixed)
    % angles: symbolic or numeric angles [alpha, beta, gamma]
    % angles_dot: symbolic or numeric angle derivatives [alpha_dot, beta_dot, gamma_dot]
    % sequence: rotation sequence (e.g., "ZYX")
    % fixed: boolean indicating fixed or rotating frame

    sequence = char(sequence);
    omega = sym(zeros(3,1)); % Initialize omega as symbolic

    if fixed
        % For fixed frame, the relationship is simpler
        for i = 1:length(sequence)
            switch sequence(i)
                case 'X'
                    omega = omega + angles_dot(i) * [1; 0; 0];
                case 'Y'
                    omega = omega + angles_dot(i) * [0; 1; 0];
                case 'Z'
                    omega = omega + angles_dot(i) * [0; 0; 1];
                otherwise
                    error("Invalid sequence");
            end
        end
    else
        % For rotating frame, the relationship is more complex
        R = eye(3);
        for i = 1:length(sequence)
            angle = angles(i);
            switch sequence(i)
                case 'X'
                    omega = omega + angles_dot(i) * R * [1; 0; 0];
                    R = R * RotX(angle);
                case 'Y'
                    omega = omega + angles_dot(i) * R * [0; 1; 0];
                    R = R * RotY(angle);
                case 'Z'
                    omega = omega + angles_dot(i) * R * [0; 0; 1];
                    R = R * RotZ(angle);
                otherwise
                    error("Invalid sequence");
            end
        end
    end

    omega = simplify(omega);
    omega = vpa(omega); % Convert to numeric
end


function R_dot = rotmDot(R, omega)

    R_dot = R * skew(omega(1), omega(2), omega(3));
end


function S = skew(x,y,z)
    S = [0, -z, y;
         z, 0, -x;
         -y, x, 0];
end

function R = anglesToRotm(angles, sequence, fixed)

    R = eye(3);

    sequence = char(sequence);

    for i = 1:length(sequence)
        angle = angles(i);
        switch sequence(i)
            case 'X'
                R = R * RotX(angle);
            case 'Y'
                R = R * RotY(angle);
            case 'Z'
                R = R * RotZ(angle);
            otherwise
                error("Invalid sequence");
        end
    end 

    if fixed
        R = R.';
    end


end

function [a1, a2] = rotmToAngles(R, sequence, fixed)

    %check if numeric
    if ~isnumeric(R)
        error("R must be numeric");
    end

    if fixed
        R = R.';
    end

    [a1 , a2] = rotm2eul(R, sequence);
end


function R = RotX(angle)
    R = [1,      0,           0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end

function R = RotY(angle)
    R = [ cos(angle), 0, sin(angle);
                0, 1,      0;
         -sin(angle), 0, cos(angle)];
end

function R = RotZ(angle)
    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
               0,           0,    1];
end

function R = Rot(angle, axis)
    switch axis
        case 'X'
            R = RotX(angle);
        case 'Y'
            R = RotY(angle);
        case 'Z'
            R = RotZ(angle);
        otherwise
            R = eye(3);
    end
end

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
