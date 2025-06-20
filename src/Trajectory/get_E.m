function E = get_E(phi)
    % get_E - Get the differential angle vector from the orientation angles
    % phi: orientation angles (3x1 vector)
    % E: differential angle vector (3x1 vector)

    % such that omega = E(phi) * phi_dot

    % ensure phi is a column vector
    if isrow(phi)
        phi = phi(:);
    end

    phi1 = phi(1); 
    phi2 = phi(2);
    %phi3 = phi(3); % phi3 is not used in the calculation

    E = [1, 0, sin(phi2);
         0, cos(phi1), -cos(phi2)*sin(phi1);
         0, sin(phi1), cos(phi1)*cos(phi2)];


    % check if phi is symbolic
    if isnumeric(phi)
        E = double(E);
    else
        E = vpa(E);
    end