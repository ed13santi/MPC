function x = w2x(w)
    x = zeros((length(w)-10)/13*10, 1);
    for i=1:(length(w)-10)/13
        x(i*10-9:i*10) = w(i*13+1:i*13+10);
    end
end

