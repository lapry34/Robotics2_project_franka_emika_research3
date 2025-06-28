function plotJointAcceleration(time, ddq_list, N)

    fig = figure('Name', 'Joint Accelerations', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 900]);

    for i = 1:N
        ax = subplot(N, 1, i);

%         yline(LIM_ddq_max(i), 'r--', 'LineWidth', 2, ...
%               'DisplayName', ['$\ddot{q}_{', num2str(i), ',\mathrm{max}}$']);
%         hold on;
%         yline(-LIM_ddq_max(i), 'r:', 'LineWidth', 2, ...
%               'DisplayName', ['$\ddot{q}_{', num2str(i), ',\mathrm{min}}$']);

        plot(time, ddq_list(i, :), 'b', 'LineWidth', 2.5, ...
             'DisplayName', ['$\ddot{q}_{', num2str(i), '}$']);
        hold on;
        if i == N
            xlabel('Time (s)', 'FontSize', 14);
        else
            set(gca, 'XTickLabel', []);
        end

        ylabel(['$\ddot{q}_{', num2str(i), '}$ (rad/s$^2$)'], ...
               'FontSize', 14, 'Interpreter', 'latex');

        grid on;
        ylim([-10, 10]);
        set(gca, 'FontSize', 14);

        lgd = legend(ax, 'Location', 'eastoutside');
        lgd.Box = 'off';
        lgd.FontSize = 14;
        lgd.Interpreter = 'latex';
    end

    sgtitle('Joint accelerations over time', ...
            'FontSize', 16, 'FontWeight', 'bold');
        
    exportgraphics(fig, 'JointAccelerations.png', 'Resolution', 300);
end
