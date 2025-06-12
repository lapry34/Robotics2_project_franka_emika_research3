function phi = get_phi(q)

        R = get_R(q);

        % we convert the rotation matrix to a XYZ Euler angles (phi1, phi2, phi3) [also known as Cardan angles]
        % determinant of R_xyz_euler = 1;
        % determinant of E_xyz_euler = -cos(phi2), so singularities occur when phi2 = pi/2 or -pi/2

        phi1 = atan2(-R(2, 3), R(3, 3)); % rotation around x-axis
        phi2 = atan2(R(1, 3), sqrt(R(1,1)^2 + R(1,2)^2));            % rotation around y-axis
        phi3 = atan2(-R(1, 2), R(1, 1)); % rotation around z-axis

        phi = [phi1; phi2; phi3];


        % check if phi is symbolic
        if isnumeric(R)
            phi = double(phi);
        else
            phi = vpa(phi);
        end
end