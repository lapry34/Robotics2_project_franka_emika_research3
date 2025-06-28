function plotErrorNorm(time, error_list, t_sing)
    % Plots norm of end-effector position error and marks singularity

    fig = figure('Name', 'End-Effector Position Error Norm', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 450]);

    plot(time, error_list, 'g', 'LineWidth', 2.5, 'DisplayName', 'Norm of Error');
    hold on;

    if ~isempty(t_sing)
        if isscalar(t_sing)
            t_sing = [t_sing];
        end

        for i = 1:numel(t_sing)
            xline(t_sing(i), '--r', 'Singularity Crossing', ...
                  'LineWidth', 2, ...
                  'FontSize', 12, ...
                  'LabelHorizontalAlignment', 'left', ...
                  'LabelVerticalAlignment', 'middle', ...
                  'HandleVisibility', 'off');
        end
    end

    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Error Norm (m)', 'FontSize', 12);
    title('Norm of End-Effector Position Error Over Time', ...
          'FontSize', 14, 'FontWeight', 'bold');

    lgd = legend('Location', 'northeastoutside', 'Interpreter', 'none');
    lgd.Box = 'off';
    lgd.FontSize = 12;

    grid on;
    set(gca, 'FontSize', 12);
   

    exportgraphics(fig, 'ErrorNorm.png', 'Resolution', 300);
end
