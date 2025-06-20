
function plotJointAngles(time, q_list)
    % Plots joint angles over time
    figure('Name', 'Joint Angles', 'NumberTitle', 'off');
    plot(time, q_list);
    xlabel('Time (s)'); ylabel('Joint Angles (rad)');
    title('Joint Angles Over Time');
    grid on;
    legend(arrayfun(@(i) sprintf('q%d', i), 1:size(q_list,1), 'UniformOutput', false));
end