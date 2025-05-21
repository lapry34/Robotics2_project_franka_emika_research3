function H = compute_H_range(q, Q_min, Q_max)
%Input:
%   q: symbolic array of joint parameter q_i
%   Q_min: array containing the minimum values various joints
%   Q_max: array containing the maximum values various joints
% Output:
%   H: the "distance" from the mid points of the joint ranges)

        H = 0;
        N = length(q);
        
        for i=1:N
            qi = q(i);
            q_mi = Q_min(i);
            q_Mi = Q_max(i);
            q_hat_i = mean([q_mi, q_Mi]);
            H = simplify(H + 1/(2*N)*((qi - q_hat_i)/(q_Mi-q_mi))^2);
        end

        H = vpa(H);
end