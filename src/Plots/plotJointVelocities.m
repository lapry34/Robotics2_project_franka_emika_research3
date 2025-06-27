function plotJointVelocities(time, dq_list, LIM_dq_max, N)
    % Plots joint velocities with bounds

    fig = figure('Name', 'Joint Velocities', ...
           'NumberTitle', 'off', ...
           'Position', [100, 100, 900, 900]);

    for i = 1:N
        ax = subplot(N, 1, i);

        yline(LIM_dq_max(i), 'r--', 'LineWidth', 2, ...
              'DisplayName', ['$\dot{q}_{', num2str(i), ',\mathrm{max}}$']);
        hold on;
        yline(-LIM_dq_max(i), 'r:', 'LineWidth', 2, ...
              'DisplayName', ['$\dot{q}_{', num2str(i), ',\mathrm{min}}$']);
        plot(time, dq_list(i, :), 'b', 'LineWidth', 2.5, ...
             'DisplayName', ['$\dot{q}_{', num2str(i), '}$']);

        if i == N
            xlabel('Time (s)', 'FontSize', 14);
        else
            set(gca, 'XTickLabel', []);
        end

        ylabel(['$\dot{q}_{', num2str(i), '}$ (rad/s)'], ...
               'FontSize', 14, 'Interpreter', 'latex');

        grid on;
        set(gca, 'FontSize', 14);

        lgd = legend(ax, 'Location', 'eastoutside');
        lgd.Box = 'off';
        lgd.FontSize = 14;
        lgd.Interpreter = 'latex';
    end

    sgtitle('Joint velocities over time', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig, 'JointVelocities.png', 'Resolution', 300);
end
