function plotJointAcceleration(time, ddq_list, LIM_ddq_max, N)
    % Plots joint Accelerations with bounds
    figure('Name', 'Joint Accelerations', 'NumberTitle', 'off');
    for i = 1:N
        subplot(N, 1, i);
        plot(time, ddq_list(i, :), 'b', 'DisplayName', ['q', num2str(i), ' Acceleration']);
        hold on;
        yline(LIM_ddq_max(i), 'r--', 'DisplayName', ['q', num2str(i), ' Max']);
        yline(-LIM_ddq_max(i), 'g--', 'DisplayName', ['q', num2str(i), ' Min']);
        xlabel('Time (s)');
        ylabel(['q', num2str(i)]);
        title(['Joint ', num2str(i), ' Acceleration (rad/s^2) Over Time']);
        grid on;
        legend;
    end
end