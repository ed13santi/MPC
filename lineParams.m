function [a,c] = lineParams(pointA, pointB)
    % given two points, returns the parameters of the line 
    % ax+(a-1)y=c
    dy = pointB(2)-pointA(2);
    dx = pointB(1)-pointA(1);
    if dx == 0
        a = 1
        c = 0
    else
        m = dy/dx; % y + a * x = c
        a = m / (1-m);
        c = pointA(2) - m * pointA(1);
    end
end

