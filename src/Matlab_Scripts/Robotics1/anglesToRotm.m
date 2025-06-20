
function R = anglesToRotm(angles, sequence, fixed)

    R = eye(3);

    sequence = char(sequence);

    for i = 1:length(sequence)
        angle = angles(i);
        switch sequence(i)
            case 'X'
                R = R * RotX(angle);
            case 'Y'
                R = R * RotY(angle);
            case 'Z'
                R = R * RotZ(angle);
            otherwise
                error("Invalid sequence");
        end
    end 

    if fixed
        R = R.';
    end


end