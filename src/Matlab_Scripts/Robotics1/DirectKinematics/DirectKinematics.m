% Import symbolic library and define variables
clear variables;
clc
syms d1 theta2

% alpha and a always constant!!!!
% Define the custom order of parameters
paramOrder = {'alpha', 'a', 'd', 'theta'};

% Define DH parameters for two joints
% Note: Each row is [alpha, a, d, theta] in the specified order
% parameters are used for trasformations in inverse order as in the book (theta, d, a, alpha)
DH = [
    pi/2, 0, d1, 0.5;   % First joint (prismatic)
    0, 0.3, 0, theta2;  % Second joint (revolute)
]

% Compute the direct kinematics
T = directKinematics(DH, paramOrder);

% Display the transformation matrix
disp('Transformation matrix T from base to end-effector:')
disp(T)

% Extract the rotation matrix from T
R = T(1:3, 1:3);
disp('Rotation matrix R:')
disp(R)

% Extract the end-effector position from T
position = T(1:3, 4);
disp('End-effector position:')
disp(position)

% Compute the orientation angles around x, y, and z axes
% Rename them as b1, b2, b3
b1 = atan2(R(3,2), R(3,3));  % Rotation around x-axis
b2 = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));  % Rotation around y-axis
b3 = atan2(R(2,1), R(1,1));  % Rotation around z-axis

%simplify angles equation
b1 = simplify(b1);
b2 = simplify(b2);
b3 = simplify(b3);

% Display the angles
disp('Orientation angle (b1) around x-axis:')
disp(b1)
disp('Orientation angle (b2) around y-axis:')
disp(b2)
disp('Orientation angle (b3) around z-axis:')
disp(b3)

function T = directKinematics(DH, paramOrder)
    % DH: nx4 matrix where each row is [p1, p2, p3, p4] in the specified order
    % paramOrder: 1x4 cell array specifying the order of DH parameters e.g {'alpha', 'a', 'd', 'theta'}
    
    % Number of joints
    n = size(DH, 1);
   
    % Map parameter names to indices
    paramMap = containers.Map({'alpha', 'a', 'd', 'theta'}, 1:4);
    paramIdx = cellfun(@(x) paramMap(x), paramOrder);
    
    % Initialize transformation matrix as identity
    T = eye(4);
    
    for i = 1:n
        % Extract DH parameters in the specified order
        alpha = DH(i, paramIdx(1));
        a = DH(i, paramIdx(2));
        d = DH(i, paramIdx(3));
        theta = DH(i, paramIdx(4));

        % Compute the transformation matrix for the current joint
        Ti = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
              sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0, sin(alpha), cos(alpha), d;
              0, 0, 0, 1];
        
        % Update the total transformation matrix
        T = T * Ti;
        T = simplify(T);
    end
end
