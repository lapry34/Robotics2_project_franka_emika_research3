function plotEndEffectorPosition(time, p_list, p_d_sym, t_sym)
    % Plots real vs nominal end-effector position over time

    fig = figure('Name', 'End-Effector Position', 'NumberTitle', 'off', ...
       'Position', [100, 100, 900, 350]);
    hold on;

    % Real vs Nominal Position: X
    plot(time, p_list(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'Real Position (X)');
    plot(time, double(subs(p_d_sym(1), t_sym, time)), 'r--', 'LineWidth', 2, 'DisplayName', 'Nominal Position (X)');

    % Real vs Nominal Position: Y
    plot(time, p_list(2, :), 'g', 'LineWidth', 2, 'DisplayName', 'Real Position (Y)');
    plot(time, double(subs(p_d_sym(2), t_sym, time)), 'm--', 'LineWidth', 2, 'DisplayName', 'Nominal Position (Y)');

    % Real vs Nominal Position: Z
    plot(time, p_list(3, :), 'c', 'LineWidth', 2, 'DisplayName', 'Real Position (Z)');
    plot(time, double(subs(p_d_sym(3), t_sym, time)), 'k--', 'LineWidth', 2, 'DisplayName', 'Nominal Position (Z)');

    xlabel('Time (s)');
    ylabel('Position (m)');
    title('End-Effector Position Over Time (X, Y, Z)');
    grid on;

    % Legenda esterna e senza bordo
    lgd = legend('Location', 'eastoutside');
    lgd.Box = 'off';

    hold off;
    exportgraphics(fig, 'EndEffectorPosition.png', 'Resolution', 300);
end
