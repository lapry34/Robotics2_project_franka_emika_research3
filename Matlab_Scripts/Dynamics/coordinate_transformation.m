

function [M_p, c_p, g_p, u_p] = coordinate_transformation(M_q, c_q, g_q, u_q, q_dot, J, J_dot, verbose)
    % M: mass matrix
    % c: coriolis matrix
    % g: gravity matrix
    % tau: torque vector
    % q: joint position vector
    % p = f(q), q = f^-1(p), we are coming from q

    %for example if q is the absolute coordinates, and p the DH coordinates
    %then then it is easier to define J_inv that goes from DH to absolute, because [1, ..0; 1, 1,...0; 1, 1, 1,...0]
    %see exam of 06 2023 ex 1

    if nargin < 8
        verbose = false;
    end

    J_inv = pinv(J); %generalized inverse can be used.

    p_dot = J*q_dot;

    M_p = J_inv' * M_q * J_inv;
    %c_p = J_inv' * c_q - M_p*J_dot*J_inv*p_dot; % J_inv*p_dot = q_dot? simplify?
    c_p = J_inv' * c_q - M_p*J_dot*q_dot;
    g_p = J_inv'*g_q;

    u_p = J_inv'*u_q;


    simplify(u_p)
    simplify(g_p)
    simplify(c_p)
    simplify(M_p)

    if verbose
        disp("M_p:");
        disp(M_p);
        disp("c_p:");
        disp(c_p);
        disp("g_p:");
        disp(g_p);
        disp("u_p:");
        disp(u_p);
    end

end

