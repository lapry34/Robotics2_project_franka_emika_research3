function H = simpler_H(q)

    % CHOSEN SINGULARITY: s2 = 0, c3 = 0, c5 = 0 
    % we return a function s.t. the minimum is at the singularity
    

    H = sin(q(2))^2 + (1 - cos(q(3)))^2 + (1 - cos(q(5)))^2;