function [a,c] = lineParams(pA, pB)
    % given two points, returns the parameters of the line 
    % ax+(a-1)y=c
    A = [pA(1)-pA(2), -1;
         pB(1)-pB(2), -1 ];
    B = [-pA(2); -pB(2)];
    tmp = A \ B;
    a = tmp(1)
    c = tmp(2)
end

