clear variables;
clc;

% Define a rotation matrix (example)
%R = [0.4330, -0.7500, 0.5000; 0.7891, 0.6124, 0.0474; -0.4330, 0.2500, 0.8660];
R = [0, 0.2588, 0.9659; 0, 0.9659, -0.2588; -1.0000, 0, 0]; %degenerate

% Compute both sets of RPY angles from the rotation matrix
[roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2] = rotationMatrixToRPY(R);

% Display the first solution in degrees
fprintf('First solution (degrees):\n');
fprintf('Roll_1: %.2f degrees\n', rad2deg(roll_1));
fprintf('Pitch_1: %.2f degrees\n', rad2deg(pitch_1));
fprintf('Yaw_1: %.2f degrees\n', rad2deg(yaw_1));

% Display the second solution in degrees
fprintf('\nSecond solution (degrees):\n');
fprintf('Roll_2: %.2f degrees\n', rad2deg(roll_2));
fprintf('Pitch_2: %.2f degrees\n', rad2deg(pitch_2));
fprintf('Yaw_2: %.2f degrees\n', rad2deg(yaw_2));

function [roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2] = rotationMatrixToRPY(R)
    % This function computes two possible sets of Roll, Pitch, and Yaw (RPY) angles
    % from a given 3x3 rotation matrix R.
    %
    % Inputs:
    % R - a 3x3 rotation matrix
    %
    % Outputs:
    % roll_1, pitch_1, yaw_1 - first solution for RPY angles
    % roll_2, pitch_2, yaw_2 - second solution for RPY angles (alternate)

    % Check if the matrix is singular (cos(pitch) == 0)
    if abs(R(3,1)) ~= 1
        % General case: non-singular
        % First solution
        pitch_1 = -asin(R(3,1));
        roll_1 = atan2(R(3,2), R(3,3));
        yaw_1 = atan2(R(2,1), R(1,1));

        % Second solution (alternative pitch)
        pitch_2 = pi - pitch_1;
        roll_2 = atan2(-R(3,2), -R(3,3));
        yaw_2 = atan2(-R(2,1), -R(1,1));
        
    else
        % Singular case: pitch is ±pi/2, ONLY ONE SOLUTION
        yaw_1 = 0;  % Arbitrary, can choose yaw = 0
        yaw_2 = 0;  % Same for the second solution

        if R(3,1) == -1
            pitch_1 = pi/2;
            roll_1 = atan2(R(1,2), R(1,3));
            pitch_2 = pi/2;
            roll_2 = roll_1;  % Same for singular case
        else
            pitch_1 = -pi/2;
            roll_1 = atan2(-R(1,2), -R(1,3));
            pitch_2 = -pi/2;
            roll_2 = roll_1;  % Same for singular case
        end
    end
end
