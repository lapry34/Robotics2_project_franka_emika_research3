
function q = franka_IK_EE_CC(O_T_EE, q7, q_actual)
    % "Case-Consistent" inverse kinematics w.r.t. End Effector Frame
    % Inputs:
    %   O_T_EE: 4x4 transformation matrix
    %   q7: joint 7 angle (redundant parameter)
    %   q_actual: 1x7 current joint angles
    % Output:
    %   q: 1x7 joint angles solution
    
    % Initialize output
    q = NaN(1, 7);
    
    % Robot parameters
    d1 = 0.3330;
    d3 = 0.3160;
    d5 = 0.3840;
    d7e = 0.2104;
    a4 = 0.0825;
    a7 = 0.0880;
    
    LL24 = 0.10666225; % a4^2 + d3^2
    LL46 = 0.15426225; % a4^2 + d5^2
    L24 = 0.326591870689; % sqrt(LL24)
    L46 = 0.392762332715; % sqrt(LL46)
    
    thetaH46 = 1.35916951803; % atan(d5/a4)
    theta342 = 1.31542071191; % atan(d3/a4)
    theta46H = 0.211626808766; % acot(d5/a4)
    
    % Joint limits
    q_max       = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]; % [rad]
    q_min       = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]; % [rad]
    
    % Check q7 bounds
    if q7 <= q_min(7) || q7 >= q_max(7)
        return;
    else
        q(7) = q7;
    end
    
    % FK for getting current case id
    c1_a = cos(q_actual(1)); s1_a = sin(q_actual(1));
    c2_a = cos(q_actual(2)); s2_a = sin(q_actual(2));
    c3_a = cos(q_actual(3)); s3_a = sin(q_actual(3));
    c4_a = cos(q_actual(4)); s4_a = sin(q_actual(4));
    c5_a = cos(q_actual(5)); s5_a = sin(q_actual(5));
    c6_a = cos(q_actual(6)); s6_a = sin(q_actual(6));
    
    % Build transformation matrices
    As_a = cell(7, 1);
    As_a{1} = [c1_a, -s1_a, 0.0, 0.0;    % O1
               s1_a,  c1_a, 0.0, 0.0;
               0.0,   0.0,  1.0,  d1;
               0.0,   0.0,  0.0, 1.0];
    
    As_a{2} = [c2_a, -s2_a, 0.0, 0.0;    % O2
               0.0,   0.0,  1.0, 0.0;
              -s2_a, -c2_a, 0.0, 0.0;
               0.0,   0.0,  0.0, 1.0];
    
    As_a{3} = [c3_a, -s3_a, 0.0,  0.0;   % O3
               0.0,   0.0, -1.0,  -d3;
               s3_a,  c3_a, 0.0,  0.0;
               0.0,   0.0,  0.0,  1.0];
    
    As_a{4} = [c4_a, -s4_a, 0.0,  a4;    % O4
               0.0,   0.0, -1.0,  0.0;
               s4_a,  c4_a, 0.0,  0.0;
               0.0,   0.0,  0.0,  1.0];
    
    As_a{5} = [1.0,  0.0,  0.0, -a4;     % H
               0.0,  1.0,  0.0,  0.0;
               0.0,  0.0,  1.0,  0.0;
               0.0,  0.0,  0.0,  1.0];
    
    As_a{6} = [c5_a, -s5_a, 0.0,  0.0;   % O5
               0.0,   0.0,  1.0,   d5;
              -s5_a, -c5_a, 0.0,  0.0;
               0.0,   0.0,  0.0,  1.0];
    
    As_a{7} = [c6_a, -s6_a, 0.0,  0.0;   % O6
               0.0,   0.0, -1.0,  0.0;
               s6_a,  c6_a, 0.0,  0.0;
               0.0,   0.0,  0.0,  1.0];
    
    Ts_a = cell(7, 1);
    Ts_a{1} = As_a{1};
    for j = 2:7
        Ts_a{j} = Ts_a{j-1} * As_a{j};
    end
    
    % Identify q6 case
    V62_a = Ts_a{2}(1:3, 4) - Ts_a{7}(1:3, 4);
    V6H_a = Ts_a{5}(1:3, 4) - Ts_a{7}(1:3, 4);
    Z6_a = Ts_a{7}(1:3, 3);
    is_case6_0 = (cross(V6H_a, V62_a)' * Z6_a <= 0);
    
    % Identify q1 case
    is_case1_1 = (q_actual(2) < 0);
    
    % IK: compute p_6
    R_EE = O_T_EE(1:3, 1:3);
    z_EE = O_T_EE(1:3, 3);
    p_EE = O_T_EE(1:3, 4);
    p_7 = p_EE - d7e * z_EE;
    
    x_EE_6 = [cos(q7 - pi/4); -sin(q7 - pi/4); 0.0];
    x_6 = R_EE * x_EE_6;
    x_6 = x_6 / norm(x_6);
    p_6 = p_7 - a7 * x_6;
    
    % IK: compute q4
    p_2 = [0.0; 0.0; d1];
    V26 = p_6 - p_2;
    
    LL26 = V26' * V26;
    L26 = sqrt(LL26);
    
    if (L24 + L46 < L26) || (L24 + L26 < L46) || (L26 + L46 < L24)
        return;
    end
    
    theta246 = acos((LL24 + LL46 - LL26) / (2.0 * L24 * L46));
    q(4) = theta246 + thetaH46 + theta342 - 2.0 * pi;
    
    if q(4) <= q_min(4) || q(4) >= q_max(4)
        return;
    end
    
    % IK: compute q6
    theta462 = acos((LL26 + LL46 - LL24) / (2.0 * L26 * L46));
    theta26H = theta46H + theta462;
    D26 = -L26 * cos(theta26H);
    
    Z_6 = cross(z_EE, x_6);
    Y_6 = cross(Z_6, x_6);
    R_6 = zeros(3, 3);
    R_6(:, 1) = x_6;
    R_6(:, 2) = Y_6 / norm(Y_6);
    R_6(:, 3) = Z_6 / norm(Z_6);
    V_6_62 = R_6' * (-V26);
    
    Phi6 = atan2(V_6_62(2), V_6_62(1));
    Theta6 = asin(D26 / sqrt(V_6_62(1)^2 + V_6_62(2)^2));
    
    if is_case6_0
        q(6) = pi - Theta6 - Phi6;
    else
        q(6) = Theta6 - Phi6;
    end
    
    if q(6) <= q_min(6)
        q(6) = q(6) + 2.0 * pi;
    elseif q(6) >= q_max(6)
        q(6) = q(6) - 2.0 * pi;
    end
    
    if q(6) <= q_min(6) || q(6) >= q_max(6)
        return;
    end
    
    % IK: compute q1 & q2
    thetaP26 = 3.0 * pi/2 - theta462 - theta246 - theta342;
    thetaP = pi - thetaP26 - theta26H;
    LP6 = L26 * sin(thetaP26) / sin(thetaP);
    
    z_6_5 = [sin(q(6)); cos(q(6)); 0.0];
    z_5 = R_6 * z_6_5;
    V2P = p_6 - LP6 * z_5 - p_2;
    
    L2P = norm(V2P);
    
    if abs(V2P(3) / L2P) > 0.999
        q(1) = q_actual(1);
        q(2) = 0.0;
    else
        q(1) = atan2(V2P(2), V2P(1));
        q(2) = acos(V2P(3) / L2P);
        if is_case1_1
            if q(1) < 0.0
                q(1) = q(1) + pi;
            else
                q(1) = q(1) - pi;
            end
            q(2) = -q(2);
        end
    end
    
    if q(1) <= q_min(1) || q(1) >= q_max(1) || ...
       q(2) <= q_min(2) || q(2) >= q_max(2)
        return;
    end
    
    % IK: compute q3
    z_3 = V2P / norm(V2P);
    Y_3 = -cross(V26, V2P);
    y_3 = Y_3 / norm(Y_3);
    x_3 = cross(y_3, z_3);
    
    c1 = cos(q(1));
    s1 = sin(q(1));
    R_1 = [c1, -s1, 0.0;
           s1,  c1, 0.0;
           0.0, 0.0, 1.0];
    
    c2 = cos(q(2));
    s2 = sin(q(2));
    R_1_2 = [c2, -s2, 0.0;
             0.0, 0.0, 1.0;
             -s2, -c2, 0.0];
    
    R_2 = R_1 * R_1_2;
    x_2_3 = R_2' * x_3;
    q(3) = atan2(x_2_3(3), x_2_3(1));
    
    if q(3) <= q_min(3) || q(3) >= q_max(3)
        return;
    end
    
    % IK: compute q5
    VH4 = p_2 + d3 * z_3 + a4 * x_3 - p_6 + d5 * z_5;
    c6 = cos(q(6));
    s6 = sin(q(6));
    R_5_6 = [c6, -s6, 0.0;
             0.0, 0.0, -1.0;
             s6,  c6, 0.0];
    R_5 = R_6 * R_5_6';
    V_5_H4 = R_5' * VH4;
    
    q(5) = -atan2(V_5_H4(2), V_5_H4(1));
    if q(5) <= q_min(5) || q(5) >= q_max(5)
        return;
    end
end