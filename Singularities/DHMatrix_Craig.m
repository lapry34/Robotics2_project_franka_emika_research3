function [T, A] = DHMatrix_Craig(arrays)
% T = DHMatrix(arrays) takes as inputs:
%   -arrays: a n-vector of vectors composed like this: [alpha a d theta]
% and outputs:
%   -T: the product of all the matrices corresponding to each vector of arrays
% Remember that:
% cos(q1 + q2) = cos(q1)*cos(q2) - sin(q1)*sin(q2)
% sin(q1 + q2) = cos(q1)*sin(q2) + cos(q2)*sin(q1)
% making use of the simplify function these are converted automatically

    T = eye(4);
    nums = size(arrays);
    
    A = cell(1,nums(1));
    
    for i = 1:nums(1)
%         line = arrays(i, :);

       
        alpha = arrays(i, 1);
        a = arrays(i, 2);
        d = arrays(i, 3);
        theta = arrays(i, 4);

        R = [          cos(theta),           -sin(theta),           0,            a;
            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
            sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d;
                                0,                     0,           0,            1 ];

        
        A{i} = R;
        T = T * R;   
    end

    if isa(T, 'sym')
        T = simplify(T);
    end
end