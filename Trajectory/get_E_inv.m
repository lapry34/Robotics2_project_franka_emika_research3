function E_inv = get_E_inv(phi)
    % get_E - Get the differential angle vector from the orientation angles
    % phi: orientation angles (3x1 vector)
    % E: differential angle vector (3x1 vector)

    % such that  phi_dot =  E(phi)_inv * omega 

    % ensure phi is a column vector
    if isrow(phi)
        phi = phi(:);
    end

    %phi1 = phi(1); %not dependent on phi1
    phi2 = phi(2);
    phi3 = phi(3);

    %check if phi2 is near pi/2 or -pi/2 to avoid singularities
    if abs(phi2) > pi/2 - 1e-6
        % Return a 3x3 matrix of NaNs to signal the error unusually
        disp('Warning: Singularities detected in get_E_inv. Returning NaN matrix.');
        E_inv = NaN(3);
        return;
    end

    % original E_inv matrix
    %E_inv = [(cos(phi3)*sin(phi2))/cos(phi2), (sin(phi2)*sin(phi3))/cos(phi2), 1; 
    %    -sin(phi3), cos(phi3), 0; 
    %    cos(phi3)/cos(phi2), sin(phi3)/cos(phi2), 0];

    E_inv = [
        cos(phi3)*tan(phi2), sin(phi3)*tan(phi2), 1;
        -sin(phi3), cos(phi3), 0;
        cos(phi3)/cos(phi2), sin(phi3)/cos(phi2), 0;
    ];

    % check if phi is symbolic
    if isnumeric(phi)
        E_inv = double(E_inv);
    else
        E_inv = vpa(E_inv);
    end