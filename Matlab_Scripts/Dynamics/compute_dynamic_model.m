function [M, c, S, g] = compute_dynamic_model(q, dq, T, U, verbose)

    if nargin < 5
        verbose = false;
    end

    if nargin < 4
        U = sym([]);
    end

    % Number of joints
    n = length(q);

    % Initialize the mass matrix M
    M = inertia_matrix_from_kinetic_energy(T, dq);
    M = simplify(M);

    % Initialize the Coriolis matrix c
    c = compute_christoffel_matrix(M, q, dq, n);
    c = simplify(c);

    if verbose
        disp('Inertia matrix M:');
        disp(M);
        
        disp('Coriolis matrix C:');
        disp(c);
    end

    % Initialize the factorization matrix S
    S = factorization_S_from_inertia_matrix(M, q, dq, verbose);
    S = simplify(S);

    % Initialize the gravity vector g
    if isempty(U)
        g = sym([]);
    else
        g = g_from_potential_energy(U, q);
        g = simplify(g);
    end

    if verbose
        disp('Gravity vector g:');
        disp(g);
    end

 
    