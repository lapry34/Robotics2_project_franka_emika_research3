
function J_geom = geometric_jacobian_deluca(DHTABLE, joints_str, N)

    
    assert(N == length(joints_str), "Mismatch between N and length of joints_str");
    
    
    %% Build the general Denavit-Hartenberg trasformation matrix
    theta = sym('theta');
    alpha = sym('alpha');
    d = sym('d');
    a = sym('a');
    
    TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
            sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
              0             sin(alpha)             cos(alpha)            d;
              0               0                      0                   1];
    
    %% Build transformation matrices for each link
    
    A = cell(1,N);
    for i = 1:N
        alpha = DHTABLE(i,1);
        a = DHTABLE(i,2);
        d = DHTABLE(i,3);
        theta = DHTABLE(i,4);
        A{i} = subs(TDH);
    end
    
    
    %% Direct kinematics
    
    
    T = eye(4);
    
    p_vec = [];
    z_vec = [];
    
    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
%     disp("p_0 = [" + join(string(p_i), "; ") + "];");
%     disp("z_0 = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_vec, p_i];
    z_vec = [z_vec, z_i];
    
    for i=1:N 
    %     disp(i)
    %     disp(A{i})
        T = T*A{i};
        T = simplify(T);
        
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);
%         disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
%         disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end
    
    %% Geometric Jacobian
    
    JP = [];
    JO = [];
    
    for i = 1:N
        p_i = p_vec(:, i);
        z_i = z_vec(:, i);
        if joints_str(i) == 'R'
            JP = [JP, cross(z_i, p_vec(:, end) - p_i)];
            JO = [JO, z_i];
        elseif joints_str(i) == 'P'
            JP = [JP, z_i];
            JO = [JO, [0; 0; 0]];
        end
    end
    
    J = [JP; JO];
    J = simplify(J);
    
    J_geom = J;

end 

