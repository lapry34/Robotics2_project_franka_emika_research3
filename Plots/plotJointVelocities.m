function plotJointVelocities(time, dq_list, LIM_dq_max, N)
    % Plots joint velocities with bounds
    figure('Name', 'Joint Velocities', 'NumberTitle', 'off');
    for i = 1:N
        subplot(N, 1, i);
        plot(time, dq_list(i, :), 'b', 'DisplayName', sprintf('q%d Velocity', i)); hold on;
        yline(LIM_dq_max(i), 'r--', 'DisplayName', sprintf('q%d Max', i));
        yline(-LIM_dq_max(i), 'g--', 'DisplayName', sprintf('q%d Min', i));
        xlabel('Time (s)'); ylabel(sprintf('q%d', i));
        title(sprintf('Joint %d Velocity (rad/s) Over Time', i));
        grid on; legend;
    end
end