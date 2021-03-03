function u = w2u(w)
    u = zeros((length(w)-10)/13*3, 1);
    for i=1:(length(w)-10)/13
        u(i*3-2:i*3) = w(i*13-2:i*13);
    end
end

