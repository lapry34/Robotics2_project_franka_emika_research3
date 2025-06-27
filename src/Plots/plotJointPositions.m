function plotJointPositions(time, q_list, LIM_q_max, LIM_q_min, N)
    % Plots joint positions with bounds
    
    fig = figure('Name', 'Joint Positions', ...
           'NumberTitle', 'off', ...
           'Position', [100, 100, 900, 900]);
    
    for i = 1:N
        ax = subplot(N, 1, i);
        
        yline(LIM_q_max(i), 'r--', 'LineWidth', 2, ...
              'DisplayName', ['$q_{', num2str(i), ',\mathrm{max}}$']);
        hold on;
        yline(LIM_q_min(i), 'r:', 'LineWidth', 2, ...
              'DisplayName', ['$q_{', num2str(i), ',\mathrm{min}}$']);
        plot(time, q_list(i, :), 'b', 'LineWidth', 2.5, ...
             'DisplayName', ['$q_{', num2str(i), '}$']);
        
        if i == N
            xlabel('Time (s)', 'FontSize', 14);
        else
            set(gca, 'XTickLabel', []);
        end
        
        ylabel(['$q_{', num2str(i), '}$ (rad)'], ...
               'FontSize', 14, 'Interpreter', 'latex');
        
        grid on;
        set(gca, 'FontSize', 14);
        
        lgd = legend(ax, 'Location', 'eastoutside');
        lgd.Box = 'off';
        lgd.FontSize = 14;
        lgd.Interpreter = 'latex';
    end
    
    sgtitle('Joint positions over time', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig, 'JointPositions.png', 'Resolution', 300);
end
