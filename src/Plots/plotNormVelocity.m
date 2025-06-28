function plotNormVelocity(time, dp_list, r_dot_sym, t_sym)
    % Plots norm of nominal and actual end-effector velocities over time

    fig = figure('Name', 'Velocity Norm Over Time', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 800, 500]);

    
    v_nominal_sampled = double(subs(r_dot_sym(1:3), t_sym, time));
    v_nom_norm = vecnorm(v_nominal_sampled);
    v_ee_norm = vecnorm(dp_list(1:3, :));

    % Plot
    plot(time, v_nom_norm, '--r', 'LineWidth', 2.2, 'DisplayName', 'Nominal Velocity');
    hold on;
    plot(time, v_ee_norm, '-b', 'LineWidth', 2.2, 'DisplayName', 'End-Effector Velocity');

    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Velocity Norm (m/s)', 'FontSize', 12);
    title('Norm of End-Effector Velocity Over Time', 'FontSize', 14, 'FontWeight', 'bold');


    lgd = legend('Location', 'northeastoutside', 'Interpreter', 'none');
    lgd.Box = 'off';
    lgd.FontSize = 12;

    grid on;
    set(gca, 'FontSize', 12);

    xlim([0, time(end)]);
    exportgraphics(fig, 'NormVelocity.png', 'Resolution', 300);
end
