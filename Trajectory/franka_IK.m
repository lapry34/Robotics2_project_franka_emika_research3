% Solutions for Franka IK when q7 is not specified

%% Solution 1: Search over q7 range
function [best_q, all_solutions] = franka_IK(O_T_EE, q_actual, n_samples)
    % Search over q7 range and find best solution based on criteria
    % Inputs:
    %   O_T_EE: 4x4 transformation matrix
    %   q_actual: 1x7 current joint angles (optional, can be [])
    %   n_samples: number of q7 samples to try (default: 20)
    % Outputs:
    %   best_q: 1x7 best joint angles
    %   all_solutions: cell array of all valid solutions
    
    if nargin < 3
        n_samples = 20;
    end
    if nargin < 2 || isempty(q_actual)
        q_actual = zeros(1, 7); % Default neutral position
    end
    
    % Joint limits for q7
    q7_min = -3.0159;
    q7_max = 3.0159;
    
    % Sample q7 values
    q7_samples = linspace(q7_min + 0.1, q7_max - 0.1, n_samples);
    
    all_solutions = {};
    best_q = [];
    best_cost = inf;
    
    for i = 1:length(q7_samples)
        q7 = q7_samples(i);
        
        % Get all 4 possible solutions for this q7
        q_all = franka_IK_EE_CC(O_T_EE, q7, q_actual);
        
        % Check each solution
        for j = 1:4
            q = q_all(j, :);
            if ~any(isnan(q))
                all_solutions{end+1} = q;
                
                % Compute cost (minimize joint motion from current position)
                if ~isempty(q_actual)
                    cost = norm(q - q_actual);
                else
                    % Alternative: minimize distance from neutral position
                    cost = norm(q);
                end
                
                if cost < best_cost
                    best_cost = cost;
                    best_q = q;
                end
            end
        end
    end
    
    if isempty(best_q)
        warning('No valid IK solution found');
        best_q = NaN(1, 7);
    end
end