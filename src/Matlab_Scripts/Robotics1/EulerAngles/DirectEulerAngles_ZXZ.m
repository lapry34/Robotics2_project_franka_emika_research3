clear variables;
clc

phi = pi/4;   % 45 degrees
theta = pi/6; % 30 degrees
psi = pi/3;   % 60 degrees

R = eulerAnglesToRotationMatrix(phi, theta, psi);
disp(R);


function R = eulerAnglesToRotationMatrix(phi, theta, psi)
    % This function computes the rotation matrix R given the Euler angles
    % in the Z-X'-Z'' convention.
    % Inputs:
    % phi   - rotation around the Z-axis (in radians)
    % theta - rotation around the X'-axis (in radians)
    % psi   - rotation around the Z''-axis (in radians)
    %
    % Output:
    % R     - resulting 3x3 rotation matrix

    % Rotation matrix around Z-axis (phi)
    Rz_phi = [cos(phi), -sin(phi), 0;
              sin(phi),  cos(phi), 0;
              0,         0,        1];

    % Rotation matrix around X'-axis (theta)
    Rx_theta = [1, 0,          0;
                0, cos(theta), -sin(theta);
                0, sin(theta),  cos(theta)];
    
    % Rotation matrix around Z''-axis (psi)
    Rz_psi = [cos(psi), -sin(psi), 0;
              sin(psi),  cos(psi), 0;
              0,         0,        1];

    % The final rotation matrix is R = Rz_psi * Rx_theta * Rz_phi
    R = Rz_psi * Rx_theta * Rz_phi;
end
