function equations = analysis_singularities(J, print_info)
    sz = size(J);
    M = min(sz);
    Minors = get_minors(J, M);
    
    determinants = sym([]);
    num_minors = length(Minors);
    if print_info == true
        disp(num_minors)
    end
    for i=1:num_minors
        
        minor = Minors{i};
        det_m = simplify(det(minor), Steps=10);
        determinants(i)= det_m;

        if print_info == true
            disp(i)
            disp(minor)
            disp(det_m)
        end

    end
    
%     fprintf("Minor determinants calculated")
    equations = determinants' == zeros(length(Minors), 1);
end