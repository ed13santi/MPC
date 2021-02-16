function out = linePars(a,b) 
    if a(1) == b(1)
    out = [-1, 0, -a(1)];
    else
    tmp = polyfit([a(1) b(1)],[a(2) b(2)],1);
    out = [-tmp(1) 1 tmp(2)];
    end
end

