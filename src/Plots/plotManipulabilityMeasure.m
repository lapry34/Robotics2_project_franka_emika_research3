function plotManipulabilityMeasure(time, H_man_list, t_sing)
    % Plots manipulability measure over time and marks singularities
    %H_man_list = H_man_list * 10; % Scale manipulability measure for better visibility
    fig = figure('Name', 'Manipulability Measure Over Time', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 800, 500]);

    % Plot
    plot(time, H_man_list, '-b', 'LineWidth', 2.2, 'DisplayName', 'Manipulability Measure');
    hold on;

    % Mark singularities if provided
    if nargin > 2 && ~isempty(t_sing)
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
    ylabel('Manipulability Measure', 'FontSize', 12);
    title('Manipulability Measure Over Time', 'FontSize', 14, 'FontWeight', 'bold');

    lgd = legend('Location', 'northeastoutside', 'Interpreter', 'none');
    lgd.Box = 'off';
    lgd.FontSize = 12;

    grid on;
    set(gca, 'FontSize', 12);

    xlim([0, time(end)]);
    exportgraphics(fig, 'ManipulabilityMeasure.png', 'Resolution', 300);
end
