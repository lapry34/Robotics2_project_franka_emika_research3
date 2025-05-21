% Test script for eulerZYZ function
% Define test angles in radians
phi = pi / 4;      % 45 degrees
theta = pi / 3;    % 60 degrees
psi = pi / 6;      % 30 degrees

% Call the eulerZYZ function
R = eulerZYZ(phi, theta, psi);

% Display the resulting rotation matrix
disp('Rotation matrix R for Euler angles (phi, theta, psi):');
disp(['phi = ', num2str(phi), ' rad, theta = ', num2str(theta), ' rad, psi = ', num2str(psi), ' rad']);
disp(R);

% Verification: Check if R is a valid rotation matrix
% Check if transpose(R) * R is approximately equal to the identity matrix
disp('Verification of orthogonality (R'' * R should be close to identity):');
disp(R' * R);

% Check if the determinant of R is close to 1
detR = det(R);
disp(['Determinant of R (should be close to 1): ', num2str(detR)]);


function R = eulerZYZ(phi, theta, psi)
    % eulerZYZ - Computes the rotation matrix for the Z-Y'-Z'' Euler angles.
    %
    % Syntax:
    %   R = eulerZYZ(phi, theta, psi)
    %
    % Inputs:
    %   phi   - Rotation angle around the initial Z axis (in radians)
    %   theta - Rotation angle around the Y' axis (in radians)
    %   psi   - Rotation angle around the final Z'' axis (in radians)
    %
    % Output:
    %   R - The resulting 3x3 rotation matrix

    % Rotation around the initial Z axis by angle phi
    Rz1 = [cos(phi), -sin(phi), 0;
           sin(phi),  cos(phi), 0;
           0,        0,        1];

    % Rotation around the Y' axis by angle theta
    Ry = [cos(theta),  0, sin(theta);
          0,           1, 0;
         -sin(theta),  0, cos(theta)];

    % Rotation around the final Z'' axis by angle psi
    Rz2 = [cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
           0,        0,        1];

    % Combined rotation matrix for Z-Y'-Z'' rotations
    R = Rz1 * Ry * Rz2;
end
