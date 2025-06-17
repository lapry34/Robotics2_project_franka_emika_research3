function p = get_p(q, orientation) 
    % get_p - Get the position vector from the joint configuration vector
    % q: joint configuration vector
    % orientation: boolean flag to indicate if orientation angles should be included in the output

    if nargin < 2
        orientation = false; % default orientation
    end

    % Ensure q is a column vector
    if isrow(q)
        q = q(:);
    end

    a4 = 0.0825; 
    a5 =-0.0825;
    a7 = 0.088;
    d1 = 0.333;
    d3 = 0.316; 
    d5 = 0.384; 
    d_e = 0.107;

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    % q7 = q(7);

    p_ee_sym = [
            a7*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - d_e*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - a4*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - a5*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + d5*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + d3*cos(q1)*sin(q2);
            a4*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - a7*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + d_e*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + a5*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - d5*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + d3*sin(q1)*sin(q2);
            d1 + a5*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + a7*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))) + d5*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)) + d_e*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))) + d3*cos(q2) - a4*cos(q3)*sin(q2);        
        ];

    % Evaluate the symbolic pee at the numeric joint angles

    % check if q is symbolic
    if isnumeric(q)
        p = double(p_ee_sym);
    else
        p = vpa(p_ee_sym);
    end
    
    if orientation
        R = get_R(q);  % compute current end-effector position
        seq_rot = 'XYZ';
        Phi = euler_rotation_inverse(seq_rot , R, 'pos');  % Compute the current XYZ Euler orientation

        if isnumeric(q)
            p = [p; double(Phi)]; % append the orientation angles to the position vector
        else
            p = [p; vpa(Phi)]; % append the orientation angles to the position vector
        end
    end

end