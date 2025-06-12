
epsi = 0.0001;
% syms epsi;
q0 = [-2;0.7*pi; sqrt(2)];
q0 = [2;pi/4; sqrt(2)];
pd = [1;-1;3];

[qf, iter] = newton_inverse(q0, pd, @fwkin_cyl, @jacobian_cyl)




function [qf, iter] = newton_inverse(q0, pd, fw_kin, jacobian)
    q = q0;
    qf = q;
    max_iter = 10;
    tol = 1e-4;
    iter = 0;
    
    while iter < max_iter
        disp("Iteration: ");
        disp(iter);
        J = jacobian(q);
        
        % Compute the error
        e = pd - fw_kin(q);
        disp("Error: ");
        disp(e);
        disp("Norm of error: ");
        disp(norm(e));
        disp("det(J): ");
        disp(det(J));
        
        if norm(e) < tol
            qf = q;
            disp("Converged");
            break;
        end

        % q = q + J \ e;
        J_inv = (pinv(J));
        q = q + J_inv * e;
        % q = simplify(q + J \ e)

        q = mod(q + pi, 2 * pi) - pi;
        disp("Angles: ");
        disp(q);
        
        iter = iter + 1;
    end
    % disp("Newton's method did not converge.");
end

function [qf, iter]  = gradient_method(q0, pd, fw_kin, jacobian)
    q = q0;
    qf = q;
    max_iter = 1000;
    tol = 1e-4;
    iter = 0;
    alpha = 0.1;
    
    while iter < max_iter
        J = jacobian(q);
        
        % Compute the error
        e = pd - fw_kin(q);
        
        if norm(e) < tol
            qf = q;
            disp("Converged");
            break;
        end

        q = q + alpha * J.' * e;

        q = mod(q + pi, 2 * pi) - pi;
        
        iter = iter + 1;
    end
    % disp("Newton's method did not converge.");
end

function p = fw_kin_2dof(q)
    l1 = 0.5;
    l2 = 0.4;
    c1 = cos(q(1));
    s1 = sin(q(1));
    c12 = cos(q(1) + q(2));
    s12 = sin(q(1) + q(2));
    x = l1 * c1 + l2 * c12;
    y = l1 * s1 + l2 * s12;
    p = [x; y];
end

function J = jacobian_2dof(q)
    l1 = 0.5;
    l2 = 0.4;
    J = [-l1 * sin(q(1)) - l2 * sin(q(1) + q(2)), -l2 * sin(q(1) + q(2));
          l1 * cos(q(1)) + l2 * cos(q(1) + q(2)),  l2 * cos(q(1) + q(2))];
end
    
function p = fw_kin_RP(q)
    q1 = q(1);
    q2 = q(2);
    x = q2*cos(q1);
    y = q2*sin(q1);
    p = [x; y];
end

function J = jacobian_RP(q)
    q1 = q(1);
    q2 = q(2);
    J = [-q2*sin(q1), cos(q1);
          q2*cos(q1), sin(q1)];
end

function q = invkin_RR(p)
    l1 = 1.5;
    l2 = 1;
    x = p(1);
    y = p(2);
    c2 = (x^2 + y^2 - (l1^2 + l2^2)) / (2 * l1 * l2);
    % s2 = -sqrt(1 - c2^2);         % elbow down 
    s2 = sqrt(1 - c2^2);        % elbow up
    q2 = atan2(s2, c2);
    q1 = atan2(y, x) - atan2(l2 * s2, l1 + l2 * c2);
    q = [q1; q2];
end
    
function p = fwkin_cyl(q)
    x = q(3)*cos(q(2));
    y = q(3)*sin(q(2));
    z = q(1);
    p = [x; y; z];
end

function J = jacobian_cyl(q)
    J = [0, -q(3)*sin(q(2)), cos(q(2));
         0,  q(3)*cos(q(2)), sin(q(2));
         1,  0,              0];
end

function q = invkin_cyl(p)
    x = p(1);
    y = p(2);
    z = p(3);
    q1 = z;
    q2 = atan2(y, x);
    q3 = sqrt(x^2 + y^2);
    q = [q1; q2; q3];
end