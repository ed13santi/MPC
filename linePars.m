function out = linePars(a,b) 
    if a(1) == b(1)
    out = [-1, 0, -a(1)]; % -x + 0y = -x_a
    else
    tmp = polyfit([a(1) b(1)],[a(2) b(2)],1); % tmp = [a b]  where y=ax+b
    out = [-tmp(1) 1 tmp(2)]; %  
    out = out/(abs(tmp(1))+1);
    end
end

