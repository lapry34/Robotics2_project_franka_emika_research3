clear variables;
clc;

% Define the Roll, Pitch, and Yaw angles (in radians)
roll = pi/4;   % 45 degrees roll
pitch = pi/6;  % 30 degrees pitch
yaw = pi/3;    % 60 degrees yaw

% Compute the rotation matrix R
R = rpyToRotationMatrix(roll, pitch, yaw);

% Display the result
fprintf("Rotation matrix R: \n");
disp(R);


function R = rpyToRotationMatrix(roll, pitch, yaw)
    % This function computes the rotation matrix R given the RPY angles:
    % roll (rotation around X-axis), pitch (rotation around Y-axis), 
    % yaw (rotation around Z-axis).
    %
    % Inputs:
    % roll  - rotation around the X-axis (in radians)
    % pitch - rotation around the Y-axis (in radians)
    % yaw   - rotation around the Z-axis (in radians)
    %
    % Output:
    % R     - resulting 3x3 rotation matrix

    % Compute the individual rotation matrices

    % Rotation matrix around X-axis (Roll)
    R_x = [1, 0, 0;
           0, cos(roll), -sin(roll);
           0, sin(roll), cos(roll)];
    
    % Rotation matrix around Y-axis (Pitch)
    R_y = [cos(pitch), 0, sin(pitch);
           0, 1, 0;
           -sin(pitch), 0, cos(pitch)];
    
    % Rotation matrix around Z-axis (Yaw)
    R_z = [cos(yaw), -sin(yaw), 0;
           sin(yaw), cos(yaw), 0;
           0, 0, 1];
    
    % The final rotation matrix is R = R_z * R_y * R_x
    R = R_z * R_y * R_x;
end
