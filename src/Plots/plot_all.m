function plot_all(  N, T, ...
                    dt, t_fin, t_sym, t_sing, ...
                    p_list, p_start, p_end, p_sing, p_d_sym, ...
                    q_list, dq_list, error_list, ...
                    LIM_dq_max, LIM_q_max, LIM_q_min, ...
                    want_acc_orient, ...
                        ddq_list, ...
                        phi_list, r_d_sym ...
    )
    % MAIN Function to call all plotting routines
    % Assumes that the following variables are in workspace or computed before:
    % N, T, ...
    % dt, t_fin, t_sym, t_sing, ...
    % p_list, p_start, p_end, p_sing, p_d_sym, ...
    % q_list, dq_list, ddq_list, error_list, ...
    % LIM_dq_max, LIM_q_max, LIM_q_min, ...

    % want_acc_orient = 
                        % 0 if do not want to plot accelerations and orientations
                        % 1 if want to plot accelerations
                        % 2 if want to plot orientations
                        % 3 if want to plot both accelerations and orientations



    % Set default for want_acc_orient if not provided
    if nargin < 18 || isempty(want_acc_orient)
        want_acc_orient = 0;
    end

    % Validate inputs based on want_acc_orient
    switch want_acc_orient
        case {1, 3}
            if nargin < 19 || isempty(ddq_list)
                error('ddq_list must be provided when want_acc_orient is 1 or 3');
            end
        otherwise
    end
    switch want_acc_orient
        case {2, 3}
            if nargin < 21 || isempty(phi_list) || isempty(r_d_sym)
                error('phi_list and r_d_sym must be provided when want_acc_orient is 2 or 3');
            end
        otherwise
    end

    % Time vector setup
%     if T > 2
%         time = 0:dt:t_fin;
%     else
%         time = 0:dt:t_fin-dt;
%     end


%     if T > 2
%         if T < 16
%             time = 0:dt:t_fin;
%         else
%             time = 0:dt:t_fin-dt; % time vector for plotting
%         end
%          
%         else
%             time = 0:dt:t_fin-dt;
%     end

    time = 0:dt:t_fin;

    % Call plotting functions
    plotEndEffectorPosition(time, p_list, p_d_sym, t_sym);
    plotErrorNorm(time, error_list, t_sing);
    plotJointAngles(time, q_list);
    plotJointVelocities(time, dq_list, LIM_dq_max, N);
    plotJointPositions(time, q_list, LIM_q_max, LIM_q_min, N);
    plotEndEffector3D(p_list, p_start, p_end, p_sing);
    plot3dMotion(p_list,p_start,p_end,p_sing,dt, true);

    if want_acc_orient == 1 || want_acc_orient == 3
        plotJointAcceleration(time, ddq_list, N);
    end
    if want_acc_orient == 2 || want_acc_orient == 3
        plotEulerAngles(time, phi_list, r_d_sym, t_sym)
    end
end