function Y = extract_Y_v4(model, a_vec, verbose)

    if nargin < 3
        verbose = false;
    end

    % model: M*ddq + n(q, dq) +...
    % a_vec: [a1; a2; ...

    Y = jacobian(model, a_vec);
    
    Y = simplify(Y);

    % Validate Y*a_vec equals model
    try
        diff_expr = simplify(Y * a_vec - model);
        assert(isempty(diff_expr) || all(diff_expr == 0), 'Validation failed: Y * a_vec != model');
    catch ME
        if verbose
            disp('Error in Y matrix computation:');
            disp('Y = ');
            disp(Y);
            disp('a_vec = ');
            disp(a_vec);
            disp('model = ');
            disp(model);
            disp('Y * a_vec = ');
            disp(Y * a_vec);
            disp('Difference = ');
            disp(diff_expr);
        end
        error('extract_Y_v4:ValidationError', 'Y matrix validation failed:\n%s', ME.message);
    end

    if verbose
        disp('Y = ');
        disp(Y)
    end;
end
