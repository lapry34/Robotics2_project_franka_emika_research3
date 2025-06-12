function E = get_E(phi)
    % get_E - Get the differential angle vector from the orientation angles
    % phi: orientation angles (3x1 vector)
    % E: differential angle vector (3x1 vector)

    % such that omega = E(phi) * phi_dot

    % ensure phi is a column vector
    if isrow(phi)
        phi = phi(:);
    end

    %phi1 = phi(1); %not dependent on phi1
    phi2 = phi(2);
    phi3 = phi(3);

    E = [0, -sin(phi3), cos(phi2)*cos(phi3);
     0, cos(phi3), cos(phi2)*sin(phi3);
     1, 0, -sin(phi2);
     ];

    % check if phi is symbolic
    if isnumeric(phi)
        E = double(E);
    else
        E = vpa(E);
    end