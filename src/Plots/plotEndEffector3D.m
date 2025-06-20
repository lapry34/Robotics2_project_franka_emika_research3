
function plotEndEffector3D(p_list, p_start, p_end, p_sing)
    % Plots 3D end-effector path with start/end and singularity
    % figure;
    figure('Name', 'End-Effector 3D Path', 'NumberTitle', 'off');
    hold on; grid on; view(3); axis equal;
    plot3(p_list(1, :), p_list(2, :), p_list(3, :)); hold on;
    plot3([p_start(1) p_end(1)], [p_start(2) p_end(2)], [p_start(3) p_end(3)], 'g--');
    xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Z Position (m)');
    title('End-Effector Position Over Time (3D)');
    scatter3(p_sing(1), p_sing(2), p_sing(3), 50, 'filled', 'MarkerFaceColor', 'k');
    scatter3(p_start(1), p_start(2), p_start(3), 10, 'filled', 'MarkerFaceColor', 'g');
    scatter3(p_end(1), p_end(2), p_end(3), 10, 'filled', 'MarkerFaceColor', 'b');
    legend('End-Effector Path', 'Start to End Path', 'Singularity Point', 'Start Point', 'End Point');
    dx = 0.5;
    xlim([p_start(1)-dx, p_start(1)+dx]);
    ylim([p_start(2)-dx, p_start(2)+dx]);
    zlim([p_start(3)-dx, p_start(3)+dx]);
    grid on;
end