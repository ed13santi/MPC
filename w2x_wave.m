function x = w2x_wave(w)
    x = zeros((length(w)-8)/10*8, 1);
    for i=1:(length(w)-8)/10
        x(i*8-7:i*8) = w(i*10-9:i*10-2);
    end
end

