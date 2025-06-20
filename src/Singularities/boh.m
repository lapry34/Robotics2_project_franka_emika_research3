%% Franka Emika – Singularity Analysis with Distance Filtering
%   Complete, self‑contained script (Matlab / Octave)

clear; clc; close all; digits 4;

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real
addpath('./Matlab_Scripts/Robotics1');

%% —— LOG FILE (append) ————————————————————————————————
logfile = 'numerical_singularities.txt';
fid = fopen(logfile,'a');
if fid == -1, error('Cannot open log file.'); end
fprintf(fid,'\n==== New run: %s ====%s',datestr(now),newline);

%% 1. SYMBOLIC JACOBIAN ----------------------------------------------------

theta      = [theta1 theta2 theta3 theta4 theta5 theta6 theta7]';
joints_str = 'RRRRRRR';

% ----- Franka limits ------------------------------------------------------
Q_max = [ 2.8973  1.7628  2.8973 -0.0698  2.8973  3.7525  2.8973]';
Q_min = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973]';

% ----- DH -----------------------------------------------------------------
alpha_i = [0; -pi/2;  pi/2;  pi/2; -pi/2;  pi/2;  pi/2; 0];
a_i     = [0; 0; 0; 0.0825; -0.0825; 0; 0.088; 0];
d_i     = [0.333; 0; 0.316; 0; 0.384; 0; 0; 0.107];

DH_table = [alpha_i, a_i, d_i, [theta; 0]];

T_ee   = DHMatrix(DH_table);
p_ee   = affine_get_translation(T_ee);
R_0_7  = affine_get_R(T_ee);

[Jl,Ja] = geometric_jacobian(p_ee, joints_str, theta, DH_table);
J_7     = simplify( blkdiag(R_0_7',R_0_7') * simplify([Jl;Ja]) );

%% 2. PARAMETERS -----------------------------------------------------------

tolerance = 1e-6;    % min σ threshold
dist      = pi/6;    % 30° in rad

%% 3. NUMERICAL FUNCTION ---------------------------------------------------

dualfprintf(fid,'Converting symbolic Jacobian to numerical function …\n');
Jf  = matlabFunction(J_7,'Vars',{theta1 theta2 theta3 theta4 theta5 theta6 theta7});
Jfn = @(q) Jf(q(1),q(2),q(3),q(4),q(5),q(6),q(7));
obj = @(q) min(svd(Jfn(q)));

%% 4. MULTI‑START OPTIMIZATION --------------------------------------------

dualfprintf(fid,'\nFinding singularities (multi‑start) …\n\n');

singularities     = [];
singularity_types = {};
opts = optimoptions('fmincon','Display','off','Algorithm','interior-point', ...
                    'TolFun',1e-12,'TolX',1e-12,'MaxFunctionEvaluations',1e4);

n_starts = 200;
for i = 1:n_starts
    % —— seed lontano ≥ dist ——
    valid=false;
    while ~valid
        q0 = Q_min + (Q_max-Q_min).*rand(7,1);
        valid = isempty(singularities) || all(vecnorm(singularities-q0,2,1) >= dist);
    end

    try
        [q_sing,~] = fmincon(obj,q0,[],[],[],[],Q_min,Q_max,[],opts);
        if min(svd(Jfn(q_sing))) < tolerance
            is_new = isempty(singularities) || all(vecnorm(singularities - q_sing,2,1) >= dist);
            if is_new
                singularities = [singularities,q_sing];
                singularity_types{end+1} = classify_singularity(q_sing,Q_min,Q_max);

                dualfprintf(fid,'Singularity %2d\n',size(singularities,2));
                print_config(q_sing,Q_min,Q_max,fid);
                dualfprintf(fid,'\n');
            end
        end
    catch; end
end

%% 5. CONFIGURAZIONI NOTE --------------------------------------------------

dualfprintf(fid,'\nSearching for specific singularity types …\n');

% 5.1 Elbow (θ4 limite)
for theta4_val = [Q_min(4);Q_max(4)]'
    obj_fixed = @(q_red) obj([q_red(1:3);theta4_val;q_red(4:6)]);
    [q_opt,fval] = fmincon(obj_fixed,zeros(6,1),[],[],[],[], ...
        [Q_min([1:3 5:7])],[Q_max([1:3 5:7])],[],opts);
    q_full = [q_opt(1:3);theta4_val;q_opt(4:6)];
    if fval < tolerance && (isempty(singularities)||all(vecnorm(singularities-q_full,2,1)>=dist))
        singularities = [singularities,q_full]; singularity_types{end+1}='Elbow';
        dualfprintf(fid,'Elbow singularity added (θ4 = %.4f rad  %.1f°).\n',theta4_val,theta4_val*180/pi);
    end
end

% 5.2 Shoulder
seeds = [0 -pi/2 0 -pi/2 0  pi/2 0;0 0 0 -pi/2 0 0 0]';
for k = 1:size(seeds,2)
    [q_opt,fval] = fmincon(obj,seeds(:,k),[],[],[],[],Q_min,Q_max,[],opts);
    if fval < tolerance && (isempty(singularities)||all(vecnorm(singularities-q_opt,2,1)>=dist))
        singularities=[singularities,q_opt]; singularity_types{end+1}='Shoulder';
        dualfprintf(fid,'Shoulder singularity added (seed %d).\n',k);
    end
end

% 5.3 Wrist (θ5 ≃ 0)
for theta5_val = [-0.1 0 0.1]
    if theta5_val<Q_min(5)||theta5_val>Q_max(5),continue;end
    obj_fixed = @(q_red) obj([q_red(1:4);theta5_val;q_red(5:6)]);
    [q_opt,fval] = fmincon(obj_fixed,zeros(6,1),[],[],[],[], ...
        [Q_min([1:4 6:7])],[Q_max([1:4 6:7])],[],opts);
    q_full = [q_opt(1:4);theta5_val;q_opt(5:6)];
    if fval < tolerance && (isempty(singularities)||all(vecnorm(singularities-q_full,2,1)>=dist))
        singularities=[singularities,q_full]; singularity_types{end+1}='Wrist';
        dualfprintf(fid,'Wrist singularity added (θ5 = %.4f rad %.1f°).\n',theta5_val,theta5_val*180/pi);
    end
end

%% 6. ANALISI DETTAGLIATA --------------------------------------------------

dualfprintf(fid,'\n\n========== SINGULARITY ANALYSIS ==========\n');
dualfprintf(fid,'Total unique singularities: %d\n\n',size(singularities,2));

for i = 1:size(singularities,2)
    q = singularities(:,i);
    J = Jfn(q); [~,S,V] = svd(J); sv = diag(S);

    dualfprintf(fid,'Singularity %d',i);
    if i<=numel(singularity_types)&&~isempty(singularity_types{i})
        dualfprintf(fid,' (%s)',singularity_types{i});
    end
    dualfprintf(fid,':\n');
    print_config(q,Q_min,Q_max,fid);
    dualfprintf(fid,'  σ = ['); dualfprintf(fid,'%.2e ',sv); dualfprintf(fid,']\n');
    dualfprintf(fid,'  Rank = %d,  κ = %.2e\n',rank(J,1e-6),cond(J));
    null_space = V(:,sv<tolerance);
    if ~isempty(null_space)
        dualfprintf(fid,'  Null dim = %d | dir = [',size(null_space,2));
        dualfprintf(fid,'%.2f ',null_space(:,1)); dualfprintf(fid,']\n');
    end
    dualfprintf(fid,'\n');
end

%% 7. SALVATAGGIO ----------------------------------------------------------
save('franka_singularities.mat','singularities','singularity_types','Jf','dist');

%% —— FUNZIONI AUSILIARIE ————————————————————————————————
function type=classify_singularity(q,Qmin,Qmax)
    tol=0.01; atMin=abs(q-Qmin)<tol; atMax=abs(q-Qmax)<tol;
    if atMin(4)||atMax(4),type='Elbow';
    elseif abs(q(5))<0.1,type='Wrist';
    elseif any(atMin(1:3))||any(atMax(1:3)),type='Shoulder';
    elseif any(atMin)|any(atMax),type='Joint limit';
    else,type='Interior'; end
end

function print_config(q,Qmin,Qmax,fid)
    dualfprintf(fid,'  θ [rad] = ['); dualfprintf(fid,'%.4f ',q); dualfprintf(fid,']\n');
    dualfprintf(fid,'  θ [deg] = ['); dualfprintf(fid,'%.1f ',q*180/pi); dualfprintf(fid,']\n');
    inside=all(q>=Qmin-1e-6 & q<=Qmax+1e-6);
    dualfprintf(fid,'  Joint limits OK: %s\n',ternary(inside,'YES','NO'));
end

function out=ternary(cond,a,b); if cond,out=a; else,out=b; end; end

function dualfprintf(fid,fmt,varargin) %#ok<*STOUT>
    fprintf(fmt,varargin{:});
    fprintf(fid,fmt,varargin{:});
end
