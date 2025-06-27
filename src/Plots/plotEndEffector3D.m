function plotEndEffector3D(p_list, p_start, p_end, p_sing)
    % Plots 3D end-effector path with start/end and singularity
    
    % Crea figura con dimensioni specifiche [left bottom width height]
    fig = figure('Name', 'End-Effector 3D Path', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 600, 600]);
             
    % Usa tiledlayout per gestire spazio titolo e grafico
    t = tiledlayout(1,1, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % Assegna titolo a tiledlayout (fuori dall'axes), in grassetto
    title(t, 'End-Effector Position Over Time (3D)', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Crea axes all’interno del tile (occupa quasi tutto lo spazio)
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); view(ax, 3); axis(ax, 'equal');

    % End-effector path (default color, solid)
    plot3(ax, p_list(1, :), p_list(2, :), p_list(3, :), 'b-', 'LineWidth', 1.5);

    % Nominal path (start to end), rosso tratteggiato
    plot3(ax, [p_start(1), p_end(1)], [p_start(2), p_end(2)], [p_start(3), p_end(3)], 'r--', 'LineWidth', 2);

    % Singolarità: 'x' rossa più grande
    scatter3(ax, p_sing(1), p_sing(2), p_sing(3), 150, 'xr', 'LineWidth', 3);

    % Start point: cerchio rosso con bordo nero
    scatter3(ax, p_start(1), p_start(2), p_start(3), 90, 'o', ...
             'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r', 'LineWidth', 1.5);

    % End point: triangolo rosso con bordo nero
    scatter3(ax, p_end(1), p_end(2), p_end(3), 90, '^', ...
             'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r', 'LineWidth', 1.5);

    % Imposta etichette con font size più grande
    fs = 14;
    xlabel(ax, 'X [m]', 'FontSize', fs);
    ylabel(ax, 'Y [m]', 'FontSize', fs);
    zlabel(ax, 'Z [m]', 'FontSize', fs);

    % Imposta limiti attorno a p_sing per zoom
    dx_in = 0.3;
    dx_fin = 0.15;
    xlim(ax, [p_sing(1)-dx_in, p_sing(1)+dx_fin]);
    ylim(ax, [p_sing(2)-dx_in, p_sing(2)+dx_fin]);
    zlim(ax, [p_sing(3)-dx_in, p_sing(3)+dx_fin]);
    
    % Legenda fuori dal grafico a destra con font size più piccolo (10)
    lgd = legend(ax, {'End-Effector Path', 'Nominal Path', 'Singularity Point', 'Start Point', 'End Point'}, ...
                 'Location', 'northeastoutside');
    lgd.FontSize = 10;
    lgd.Box = 'off';
    exportgraphics(fig, 'EndEffector3D.png', 'Resolution', 300);
end
