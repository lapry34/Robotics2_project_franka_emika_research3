clear variables;
clc

% Define a rotation matrix R
R = [0.5, -0.866, 0;
     0.866, 0.5, 0;
     0, 0, 1];

% Find the two sets of Euler angles phi, theta, psi
[phi_1, theta_1, psi_1, phi_2, theta_2, psi_2] = rotationMatrixToEulerAngles(R);

% Display the first set of results in degrees
fprintf('First solution:\n');
fprintf('phi_1 = %.2f degrees\n', rad2deg(phi_1));
fprintf('theta_1 = %.2f degrees\n', rad2deg(theta_1));
fprintf('psi_1 = %.2f degrees\n', rad2deg(psi_1));

% Display the second set of results in degrees
fprintf('\nSecond solution:\n');
fprintf('phi_2 = %.2f degrees\n', rad2deg(phi_2));
fprintf('theta_2 = %.2f degrees\n', rad2deg(theta_2));
fprintf('psi_2 = %.2f degrees\n', rad2deg(psi_2));


function [phi_1, theta_1, psi_1, phi_2, theta_2, psi_2] = rotationMatrixToEulerAngles(R)
    % This function computes the two possible sets of Euler angles phi, theta, psi
    % from a given rotation matrix R, assuming the Z-X'-Z'' Euler angle convention.
    %
    % Inputs:
    % R - a 3x3 rotation matrix
    %
    % Outputs:
    % phi_1, theta_1, psi_1 - first set of solutions
    % phi_2, theta_2, psi_2 - second set of solutions

    % Check if the rotation matrix is singular
    if abs(R(3,1)) ~= 1
        % General case (non-singular)
        % First solution
        theta_1 = acos(R(3,1));
        phi_1 = atan2(R(2,1), R(1,1));
        psi_1 = atan2(R(3,2), -R(3,3));

        % Second solution
        theta_2 = -theta_1;
        phi_2 = atan2(-R(2,1), -R(1,1));
        psi_2 = atan2(-R(3,2), R(3,3));
    else
        % Singular case: theta = 0 or theta = pi
        psi_1 = 0; % Arbitrary, since psi and phi are coupled in this case
        psi_2 = 0; % Same as above
        if R(3,1) == -1
            theta_1 = pi;
            phi_1 = atan2(-R(1,2), R(2,2)); % phi_1 + psi_1 = atan2(-R(1,2), R(2,2))
            theta_2 = pi;  % Second solution is identical in this case
            phi_2 = phi_1;
        else
            theta_1 = 0;
            phi_1 = atan2(R(1,2), R(2,2));  % phi_1 - psi_1 = atan2(R(1,2), R(2,2))
            theta_2 = 0;  % Second solution is identical in this case
            phi_2 = phi_1;
        end
    end
end
