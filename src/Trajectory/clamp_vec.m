function c = clamp_vec(v, min_val_v, max_val_v)
%CLAMP_VEC Clamp a vector between two vectors element-wise.

%   c = clamp_vec(v, min_val_v, max_val_v)
%   Clamps the vector v between min_val_v and max_val_v element-wise.
%   Inputs:
%       v: vector to be clamped
%       min_val_v: vector of minimum values
%       max_val_v: vector of maximum values
%   Outputs:
%       c: clamped vector

    assert (length(v) == length(min_val_v) && length(v) == length(max_val_v), ...
        'Input vectors must have the same length.');

    c = zeros(size(v));
    for i = 1:length(v)
        c(i) = clamp(v(i), min_val_v(i), max_val_v(i));
    end
end