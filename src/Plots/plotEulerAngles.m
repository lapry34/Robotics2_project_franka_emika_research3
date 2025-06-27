function plotEulerAngles(time, phi_list, r_d_sym, t_sym)
    % plotEulerAngles plots the Euler angles from the given phi_list and
    % compares them with the nominal angles defined in r_d_sym.

    if size(phi_list, 1) ~= 3
        error('phi_list must be a 3xN matrix.');
    end

    fig = figure('Name', 'Euler Angles', 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);

    % Crea layout con spaziatura ridotta
    t = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    for i = 1:3
        ax = nexttile;

        plot(time, rad2deg(phi_list(i, :)), 'b', 'LineWidth', 2, ...
            'DisplayName', ['$\phi_{' num2str(i) '}$ real']);
        hold on;
        plot(time, rad2deg(double(subs(r_d_sym(i + 3), t_sym, time))), ...
            'r--', 'LineWidth', 2, 'DisplayName', ['$\phi_{' num2str(i) '}$ nominal']);

        if i == 2
            yline(rad2deg(pi/2), 'r--', 'LineWidth', 2, ...
                'DisplayName', ['$\phi_{' num2str(i) '}$ Max']);
            yline(rad2deg(-pi/2), 'g--', 'LineWidth', 2, ...
                'DisplayName', ['$\phi_{' num2str(i) '}$ Min']);
        end

        % Label asse x solo per l'ultimo subplot
        if i == 3
            xlabel('Time (s)', 'FontSize', 12);
        else
            set(gca, 'XTickLabel', []);
        end

        % Label asse y per tutti
        ylabel(['$\phi_{' num2str(i) '}$ (deg)'], 'Interpreter', 'latex', 'FontSize', 12);

        grid on;
        set(ax, 'FontSize', 12);

        lgd = legend(ax, 'Location', 'eastoutside', 'Interpreter', 'latex');
        lgd.Box = 'off';
        lgd.FontSize = 12;
    end

    % Titolo generale
    title(t, 'Euler angles over time', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig, 'EulerAngles.png', 'Resolution', 300);
end
