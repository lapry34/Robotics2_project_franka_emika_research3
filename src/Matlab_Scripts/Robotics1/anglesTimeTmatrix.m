
function T = anglesTimeTmatrix(angles, sequence, fixed)
    % angles: symbolic or numeric angles [alpha, beta, gamma]
    % sequence: rotation sequence (e.g., "ZYX")
    % fixed: boolean indicating fixed or rotating frame

    sequence = char(sequence);
    
    R1 = eye(3);
    R2 = Rot(angles(1), sequence(1));
    R3 = R2 * Rot(angles(2), sequence(2));

    n1 = numberAxis(sequence(1));
    n2 = numberAxis(sequence(2));
    n3 = numberAxis(sequence(3));

    if fixed 
        T = [R3(:,n3), R2(:,n2), R1(:,n1)];
    else 
        T = [R1(:,n1), R2(:,n2), R3(:,n3)];
    end

end

function n = numberAxis(axis)
    switch axis
        case 'X'
            n = 1;
        case 'Y'
            n = 2;
        case 'Z'
            n = 3;
        otherwise
            n = 0;
    end
end
