clear variables;
clc

R = [
    0, 0.5, -sqrt(3)/2;
    -1, 0, 0;
    0, sqrt(3)/2, 0.5;
    ];

sequence = "ZYX";

[a1, a2] = rotm2eul(R, sequence);

fprintf("degrees:\n")
fprintf("First solution of euler angles: ")
disp(rad2deg(a1));
fprintf("Second solution of euler angles: ")
disp(rad2deg(a2));