function c = clamp(x, minVal, maxVal)
    %CLAMP Clamp the value of x between minVal and maxVal    

    c = min(max(x, minVal), maxVal); % clamp x between minVal and maxVal   
end 