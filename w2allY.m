function yVec = w2allY(w)
    N = (length(w)-8)/10;
    yVec = zeros(N,1);
    for i=1:N
        yVec(i) = w(10*i+3);
    end
end

