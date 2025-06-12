function R = get_R(q)
% get_R - Get the rotation matrix from the joint configuration vector

   % q: joint angles (1x6 vector)

    % Ensure q is a column vector
    if isrow(q)
        q = q(:);
    end


    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);


    R_sym = [
    [cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))), - cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))), - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))];
    [sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))),   cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))),   cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))];
    [                                                                                                                                                                                                                                            sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) + cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))),                                                                                                                                                                                                                                               cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))),                                                                                                                                                   sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))];
    ];

        % check if q is symbolic
    if isnumeric(q)
        R = double(R_sym);
    else
        R = vpa(R_sym);
    end
end