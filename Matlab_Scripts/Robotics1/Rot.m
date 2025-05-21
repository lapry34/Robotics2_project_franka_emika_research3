function R = Rot(angle, axis)
    switch axis
        case 'X'
            R = RotX(angle);
        case 'Y'
            R = RotY(angle);
        case 'Z'
            R = RotZ(angle);
        otherwise
            R = eye(3);
    end
end