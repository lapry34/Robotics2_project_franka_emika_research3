function [a1, a2] = rotmToAngles(R, sequence, fixed)

    %check if numeric
    if ~isnumeric(R)
        error("R must be numeric");
    end

    if fixed
        R = R.';
    end

    [a1 , a2] = rotm2eul(R, sequence);
end