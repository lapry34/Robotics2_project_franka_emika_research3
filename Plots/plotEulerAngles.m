function plotEulerAngles(time, phi_list, r_d_sym, t_sym)
    % plotEulerAngles plots the Euler angles from the given phi_list and
    % compares them with the nominal angles defined in r_d_sym.

    % Ensure phi_list is a 3xN matrix
    if size(phi_list, 1) ~= 3
        error('phi_list must be a 3xN matrix.');
    end

    % Generate time vector based on the number of columns in phi_list
    figure('Name', 'Euler Angles', 'NumberTitle', 'off');
    for i = 1:3
        subplot(3, 1, i);
        plot(time, rad2deg(phi_list(i, :)), 'b', 'DisplayName', ['Real Phi', num2str(i)]);
        hold on;
        plot(time, rad2deg(double(subs(r_d_sym(i + 3), t_sym, time))), 'r--', 'DisplayName', ['Nominal Phi', num2str(i)]);
        if i == 2 % singularity at phi2 = +- pi/2
            yline(rad2deg(pi/2), 'r--', 'DisplayName', ['Phi', num2str(i), ' Max']);
            yline(rad2deg(-pi/2), 'g--', 'DisplayName', ['Phi', num2str(i), ' Min']);
        end
        xlabel('Time (s)');
        ylabel(['Phi', num2str(i), ' (deg)']);
        title(['Orientation Angle Phi', num2str(i), ' Over Time']);
        grid on;
        legend;
    end

end