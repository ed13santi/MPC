function u = w2u(w)
    u = zeros((length(w)-8)/10*2, 1);
    for i=1:(length(w)-8)/10
        u(i*2-1:i*2) = w(i*10-1:i*10);
    end
end

