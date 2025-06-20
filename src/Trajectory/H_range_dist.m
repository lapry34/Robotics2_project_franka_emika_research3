function H = H_range_dist(q)
%Input:
%   q: symbolic array of joint parameter q_i
% Output:
%   H: the "distance" from the mid points of the joint ranges)

        Q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]'; % [rad]
        Q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, -0.5445, -3.0159]'; % [rad]
        H = 0;
        N = length(q);
        
        for i=1:N
            qi = q(i);
            q_mi = Q_min(i);
            q_Mi = Q_max(i);
            q_hat_i = mean([q_mi, q_Mi]);


            H = H + 1/(2*N)*((qi - q_hat_i)/(q_Mi-q_mi))^2;            
        end

    if isnumeric(q)
        H = double(H);
    else
        H = vpa(H);
    end
end