function x = w2x(w)
    N = (length(w)-8)/10;
    x = zeros(N*8, 1);
    for i=1:N
        x(i*8-7:i*8) = w(i*10+1:i*10+8);
    end
end

