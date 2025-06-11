%% Franka Emika – Singularity Analysis with Distance Filtering
%   Complete, self‑contained script (Matlab / Octave)
%   Modifiche principali:
%     • aggiunta variabile "dist = pi/6" (≈ 30°)
%     • riconoscimento / rigenerazione seed basato su distanza euclidea
%     • filtro duplicati in *ogni* sezione con soglia "dist"
% -------------------------------------------------------------------------

clear; clc; close all; digits 4;

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
syms a4 a5 a7 real
syms d1 d3 d5 d_e real

addpath('./Matlab_Scripts/Robotics1');

%% 1. SYMBOLIC JACOBIAN ----------------------------------------------------

theta   = [theta1 theta2 theta3 theta4 theta5 theta6 theta7]';
N       = 7;
joints_str = 'RRRRRRR';

% ----- Franka Emika (research 3) limits ----------------------------------
Q_max = [ 2.8973  1.7628  2.8973 -0.0698  2.8973  3.7525 2.8973]';
Q_min = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973]';
Q_dot_max   = [2.62 2.62 2.62 2.62 5.26 4.18 5.26]';
Q_ddot_max  = 10*ones(1,N);
Q_dddot_max = 5000*ones(1,N);

% ----- DH parameters ------------------------------------------------------
alpha_i = [0; -pi/2;  pi/2;  pi/2; -pi/2;  pi/2;  pi/2; 0];
a_i     = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
d_i     = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];
theta_i = [theta; 0];

DH_table = [alpha_i, a_i, d_i, theta_i];

T_ee   = DHMatrix(DH_table);
p_ee   = affine_get_translation(T_ee);
R_0_7  = affine_get_R(T_ee);

% — Geometric Jacobian in RF_0
[Jl,Ja] = geometric_jacobian(p_ee, joints_str, theta, DH_table);
J_geom  = simplify([Jl; Ja]);

% — Express Jacobian in RF_7
R_7_0 = R_0_7';
R_j   = blkdiag(R_7_0, R_7_0);
J_7   = simplify(R_j * J_geom);

%% 2. PARAMETERS FOR NUMERICAL SEARCH --------------------------------------

tolerance = 1e-6;    % singularity threshold on min singular value

% Distanza minima fra due singolarità (≈ 30°)
dist      = pi/6;

%% 3. NUMERICAL FUNCTION ----------------------------------------------------

fprintf('Converting symbolic Jacobian to numerical function ...\n');
J_7_func  = matlabFunction(J_7,'Vars',{theta1 theta2 theta3 theta4 theta5 theta6 theta7});
compute_J7 = @(q) J_7_func(q(1),q(2),q(3),q(4),q(5),q(6),q(7));

% — Objective: minimize the smallest singular value
objective_svd = @(q) min(svd(compute_J7(q)));

%% 4. MULTI‑START OPTIMIZATION ---------------------------------------------

fprintf('\nFinding singularities using multi‑start optimization ...\n\n');

singularities     = [];
singularity_types = {};

options = optimoptions('fmincon', ...
    'Display','off', ...
    'Algorithm','interior-point', ...
    'TolFun',1e-12,'TolX',1e-12, ...
    'MaxFunctionEvaluations',1e4);

n_starts = 200;
for i = 1:n_starts
    %% Seed generation fuori dalla "sfera" di raggio dist
    valid_seed = false;
    while ~valid_seed
        q0 = Q_min + (Q_max - Q_min).*rand(7,1);
        valid_seed = isempty(singularities) || all(vecnorm(singularities - q0,2,1) >= dist);
    end

    try
        [q_sing,fval] = fmincon(objective_svd,q0,[],[],[],[],Q_min,Q_max,[],options);
        J = compute_J7(q_sing);
        min_sv = min(svd(J));

        if min_sv < tolerance
            % — verifica "nuova" singolarità
            is_new = isempty(singularities) || all(vecnorm(singularities - q_sing,2,1) >= dist);
            if is_new
                singularities     = [singularities, q_sing];
                singularity_types{end+1} = classify_singularity(q_sing,Q_min,Q_max);

                fprintf('Singularity %2d  min(svd)=%.2e\n',size(singularities,2),min_sv);
                fprintf('  θ [deg]= ['); fprintf('%.1f ',q_sing*180/pi); fprintf(']\n\n');
            end
        end
    catch
        % ignore failures
    end
end

%% 5. SPECIFIC KNOWN CONFIGURATIONS ----------------------------------------
fprintf('\nSearching for specific singularity types ...\n\n');

% ---------- 5.1 Elbow (theta4 at limits) ----------------------------------
fprintf('Checking elbow singularities ...\n');
for theta4_val = [Q_min(4); Q_max(4)]'
    obj_fixed = @(q_red) objective_svd([q_red(1:3); theta4_val; q_red(4:6)]);
    q0_red = zeros(6,1);

    [q_opt,fval] = fmincon(obj_fixed,q0_red,[],[],[],[], ...
        [Q_min([1:3 5:7])], [Q_max([1:3 5:7])], [], options);

    q_full = [q_opt(1:3); theta4_val; q_opt(4:6)];
    if fval < tolerance && (isempty(singularities) || all(vecnorm(singularities - q_full,2,1) >= dist))
        singularities     = [singularities, q_full];
        singularity_types{end+1} = 'Elbow';
        fprintf('  θ4 = %.1f°  elbow singularity added.\n',theta4_val*180/pi);
    end
end

% ---------- 5.2 Shoulder configs -----------------------------------------
fprintf('\nChecking shoulder singularities ...\n');
shoulder_seeds = [ 0 -pi/2  0 -pi/2 0  pi/2 0; ...
                   0   0    0 -pi/2 0   0   0 ]';
for k = 1:size(shoulder_seeds,2)
    q_init = shoulder_seeds(:,k);
    [q_opt,fval] = fmincon(objective_svd,q_init,[],[],[],[],Q_min,Q_max,[],options);
    if fval < tolerance && (isempty(singularities) || all(vecnorm(singularities - q_opt,2,1) >= dist))
        singularities     = [singularities, q_opt];
        singularity_types{end+1} = 'Shoulder';
        fprintf('  Shoulder singularity added (seed %d).\n',k);
    end
end

% ---------- 5.3 Wrist (theta5 ≃ 0) ---------------------------------------
fprintf('\nChecking wrist singularities ...\n');
for theta5_val = [-0.1 0 0.1]
    if theta5_val < Q_min(5) || theta5_val > Q_max(5), continue; end
    obj_fixed = @(q_red) objective_svd([q_red(1:4); theta5_val; q_red(5:6)]);
    q0_red = zeros(6,1);
    [q_opt,fval] = fmincon(obj_fixed,q0_red,[],[],[],[], ...
        [Q_min([1:4 6:7])], [Q_max([1:4 6:7])], [], options);
    q_full = [q_opt(1:4); theta5_val; q_opt(5:6)];
    if fval < tolerance && (isempty(singularities) || all(vecnorm(singularities - q_full,2,1) >= dist))
        singularities     = [singularities, q_full];
        singularity_types{end+1} = 'Wrist';
        fprintf('  Wrist singularity added (θ5 = %.2f rad).\n',theta5_val);
    end
end

%% 6. ANALYSIS -------------------------------------------------------------
fprintf('\n\n========== SINGULARITY ANALYSIS ==========');
fprintf('\nTotal unique singularities found: %d\n\n',size(singularities,2));

for i = 1:size(singularities,2)
    q = singularities(:,i);
    J = compute_J7(q);
    [~,S,V] = svd(J);
    sv = diag(S);

    fprintf('Singularity %d',i);
    if i <= numel(singularity_types) && ~isempty(singularity_types{i})
        fprintf(' (Type: %s)',singularity_types{i});
    end
    fprintf(':\n');
    fprintf('  θ [deg] = ['); fprintf('%.1f ',q*180/pi); fprintf(']\n');
    fprintf('  σ = ['); fprintf('%.2e ',sv); fprintf(']\n');
    fprintf('  Rank = %d,  κ = %.2e\n',rank(J,1e-6),cond(J));

    null_space = V(:,sv < tolerance);
    if ~isempty(null_space)
        fprintf('  Null dim = %d | dir = [',size(null_space,2));
        fprintf('%.2f ',null_space(:,1));
        fprintf(']\n');
    end
    fprintf('\n');
end

%% 7. VISUALIZATION (mappa singolarità) ------------------------------------
%  Codice originale rimane invariato; usa la matrice "singularities" già
%  filtrata con distanza "dist".

%  ( ... )

%% 8. SALVATAGGIO ----------------------------------------------------------
fprintf('Saving results ...\n');
save('franka_singularities.mat','singularities','singularity_types','J_7_func','dist');
fprintf('Results saved in franka_singularities.mat\n');

%% 9. HELPER FUNCTION ------------------------------------------------------
function type = classify_singularity(q,Q_min,Q_max)
    tol = 0.01;
    at_min = abs(q - Q_min) < tol;
    at_max = abs(q - Q_max) < tol;

    if at_min(4) || at_max(4)
        type = 'Elbow';
    elseif abs(q(5)) < 0.1
        type = 'Wrist';
    elseif any(at_min(1:3)) || any(at_max(1:3))
        type = 'Shoulder';
    elseif any(at_min) || any(at_max)
        type = 'Joint limit';
    else
        type = 'Interior';
    end
end
