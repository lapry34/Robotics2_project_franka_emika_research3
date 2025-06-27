function plotJointAngles(time, q_list)
    % Plots joint angles over time

    fig = figure('Name', 'Joint Angles', 'NumberTitle', 'off', 'Position', [100, 100, 900, 450]);
    hold on;

    % Traccia ogni giunto con linea più spessa
    for i = 1:size(q_list, 1)
        plot(time, q_list(i, :), 'LineWidth', 2, 'DisplayName', sprintf('q%d', i));
    end

    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Joint Angles (rad)', 'FontSize', 12);
    title('Joint Angles Over Time', 'FontSize', 14);
    grid on;

    % Imposta la dimensione dei tick degli assi
    set(gca, 'FontSize', 12);

    % Legenda esterna senza bordo e con scritte più grandi
    lgd = legend('Location', 'eastoutside');
    lgd.Box = 'off';
    lgd.FontSize = 12;

    hold off;
    exportgraphics(fig, 'JointAngles.png', 'Resolution', 300);
end
