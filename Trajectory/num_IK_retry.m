function q_i = num_IK_retry(r, q_est, verbose)
% NUM_IK  Inverse kinematics con gestione dei limiti articolari e ri-tentativi.
%
%   q_i = NUM_IK(r)                     % usa seed centrale
%   q_i = NUM_IK(r,q_est)               % usa seed specificato
%   q_i = NUM_IK(r,q_est,verboseFlag)   % verbose = true/false
%
%   Se non converge o esce dai limiti, riparte da un nuovo seed random
%   entro gli stessi limiti, fino a max_attempts volte.
%
%   Dipende dalle funzioni utente:
%       get_p(q,orientation)  – posizione (e, opz., orientazione)
%       get_J(q,orientation)  – Jacobiano corrispondente
%
%   Bombolone alla Crema – giugno 2025
% -------------------------------------------------------------------------

    if nargin < 3,  verbose = false;             end
    if nargin < 2,  q_est  = [];                 end   %#ok<*NBRAK>
    
    % ---------------- PARAMETRI ----------------
    LIM_q_max = [+2.7437; +1.7837; +2.9007; -0.1518; +2.8065; +4.5169; +3.0159];
    LIM_q_min = [-2.7437; -1.7837; -2.9007; -3.0421; -2.8065; -0.5445; -3.0159];
    
    alpha     = 0.3;            % passo (learning-rate)
    tol       = 1e-4;           % tolleranza
    max_iter  = 30;             % iterazioni per tentativo
    max_attempts = 20;          % tentativi complessivi
    
    % seed iniziale: se non dato → punto medio dei limiti
    if isempty(q_est)
        q_est = 0.5*(LIM_q_min + LIM_q_max);
    end
    
    % posizione vs posizione+orientamento
    orientation = (length(r) ~= 3);
    
    % ---------- LOOP sui tentativi ----------
    found = false;
    for tryId = 1:max_attempts
        
        % seed per questo tentativo
        if tryId == 1
            q_i = clamp(q_est, LIM_q_min, LIM_q_max);
        else
            q_i = LIM_q_min + rand(7,1).*(LIM_q_max - LIM_q_min);
            if verbose, fprintf('[%2d] nuovo seed casuale.\n', tryId); end
        end
        
        % ---------- Newton / GD interno ----------
        for iter = 1:max_iter
            error = r - get_p(q_i, orientation);
            if norm(error) < tol
                found = true;  break;       % convergenza
            end
            
            J_curr      = get_J(q_i, orientation);
            min_singval = svds(J_curr, 1, 'smallest');
            
            if min_singval > 1e-8 && min_singval < 1e-4     % vicino a sing.
                if verbose
                    fprintf('[%2d:%2d] GD step, σ_min = %.2e\n', tryId, iter, min_singval);
                end
                q_i = q_i + alpha * (J_curr.' * error);     % gradiente
            else
                if verbose
                    fprintf('[%2d:%2d] pinv step, σ_min = %.2e\n', tryId, iter, min_singval);
                end
                q_i = q_i + alpha * (pinv(J_curr) * error); % pseudo-inv
            end
            
            % --------- proiezione sui limiti ---------
            q_i = clamp(q_i, LIM_q_min, LIM_q_max);
        end % iter
        
        % se finito dentro limiti e convergente → stop
        if found
            if verbose
                fprintf('Convergenza in %d tentativi, %d iterazioni.\n', tryId, iter);
            end
            return
        end
    end
    
    % ---------- se siamo qui, fallimento ----------
    warning('num_IK:NoConvergence', ...
            'Non è stato trovato nessun IK entro i limiti dopo %d tentativi.', max_attempts);
    q_i = NaN(7,1);
end
% --------------- helper ------------------
function q = clamp(q, qmin, qmax)
    q = min(max(q, qmin), qmax);
end