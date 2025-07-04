function plotNormAcceleration(time, ddp_list, r_ddot_sym, t_sym)
    % Plots joint velocities with bounds
    fig = figure('Name', 'Acceleration Magnitude Over Time', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 800, 500]);

    a_nominal_sampled = double(subs(r_ddot_sym(1:3), t_sym, time));
    a_nom_norm = vecnorm(a_nominal_sampled);
    a_ee_norm = vecnorm(ddp_list(1:3,:));
    
    plot(time, a_nom_norm, 'r--', 'LineWidth', 2.2,'DisplayName', 'Nominal Acceleration' );
    hold on
    plot(time, a_ee_norm, 'b-', 'LineWidth', 2.2, 'DisplayName', 'End-Effector Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration Norm (m/s^2)');
    title('Norm of End-Effector Acceleration Over Time', 'FontSize', 14, 'FontWeight', 'bold');


    lgd = legend('Location', 'northeastoutside', 'Interpreter', 'none');
    lgd.Box = 'off';
    lgd.FontSize = 12;
    
    grid on;
    set(gca, 'FontSize', 12);
    xlim([0, time(end)]);
    
     exportgraphics(fig, 'NormAcceleration.png', 'Resolution', 300);
end
