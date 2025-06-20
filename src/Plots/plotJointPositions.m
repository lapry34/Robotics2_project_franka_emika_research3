
function plotJointPositions(time, q_list, LIM_q_max, LIM_q_min, N)
    % Plots joint positions with bounds
    figure('Name', 'Joint Positions', 'NumberTitle', 'off');
    for i = 1:N
        subplot(N, 1, i);
        plot(time, q_list(i, :), 'b', 'DisplayName', sprintf('q%d Position', i)); hold on;
        yline(LIM_q_max(i), 'r--', 'DisplayName', sprintf('q%d Max', i));
        yline(LIM_q_min(i), 'g--', 'DisplayName', sprintf('q%d Min', i));
        xlabel('Time (s)'); ylabel(sprintf('q%d', i));
        title(sprintf('Joint %d Position (rad) Over Time', i));
        grid on; legend;
    end
end