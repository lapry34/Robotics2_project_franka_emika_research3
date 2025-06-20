
function plotEndEffectorPosition(time, p_list, p_d_sym, t_sym)
    % Plots real vs nominal end-effector position over time
    % figure;
    figure('Name', 'End-Effector Position', 'NumberTitle', 'off');
    hold on;
    plot(time, p_list(1, :), 'b', 'DisplayName', 'Real Position (X)');
    plot(time, double(subs(p_d_sym(1), t_sym, time)), 'r--', 'DisplayName', 'Nominal Position (X)');
    plot(time, p_list(2, :), 'g', 'DisplayName', 'Real Position (Y)');
    plot(time, double(subs(p_d_sym(2), t_sym, time)), 'm--', 'DisplayName', 'Nominal Position (Y)');
    plot(time, p_list(3, :), 'c', 'DisplayName', 'Real Position (Z)');
    plot(time, double(subs(p_d_sym(3), t_sym, time)), 'k--', 'DisplayName', 'Nominal Position (Z)');
    xlabel('Time (s)'); ylabel('Position (m)');
    title('End-Effector Position Over Time (X, Y, Z)');
    legend; grid on;
end