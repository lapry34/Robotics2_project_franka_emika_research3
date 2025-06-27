function plotErrorNorm(time, error_list, t_sing)
    % Plots norm of end-effector position error and marks singularity
    
    fig = figure('Name', 'End-Effector Position Error Norm', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 450]);  % ← salvato come fig
       
    plot(time, error_list, 'g', 'LineWidth', 2.5, 'DisplayName', 'Norm of Error');
    hold on;
    
    xline(t_sing, '--r', 'Singularity Crossing', ...
          'LineWidth', 2, ...
          'FontSize', 12, ...
          'LabelHorizontalAlignment', 'left', ...
          'LabelVerticalAlignment', 'middle', ...
          'HandleVisibility', 'off');

    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Error Norm (m)', 'FontSize', 12);
    title('Norm of End-Effector Position Error Over Time', 'FontSize', 14, 'FontWeight', 'bold');

    lgd = legend('Interpreter', 'none');
    lgd.Box = 'off';
    lgd.FontSize = 12;

    grid on;
    set(gca, 'FontSize', 12);
    
    % Salva la figura come PNG
    exportgraphics(fig, 'ErrorNorm.png', 'Resolution', 300);
end
