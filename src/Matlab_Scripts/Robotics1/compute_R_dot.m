function R_dot = compute_R_dot(R, angles, angles_dot)
    R_dot = sym(zeros(3,3));

    for i = 1:length(angles)
        R_dot = R_dot + diff(R, angles(i)) * angles_dot(i);
    end

    R_dot = simplify(R_dot);
end