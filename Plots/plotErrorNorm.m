function plotErrorNorm(time, error_list, t_sing)
    % Plots norm of end-effector position error and marks singularity
    figure('Name', 'End-Effector Position Error Norm', 'NumberTitle', 'off');
    plot(time, error_list, 'g', 'DisplayName', 'Norm of Error');
    xlabel('Time (s)'); ylabel('Error Norm (m)');
    title('Norm of End-Effector Position Error Over Time');
    xline(t_sing, '--r', 'Singularity Crossing', ...
          'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle', ...
          'HandleVisibility', 'off');
    legend; grid on;
end