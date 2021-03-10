function yVec = w2allY(w)
    N = (length(w)-8)/10;
    yVec = zeros(N+1,1);
    for i=0:N
        yVec(i+1) = w(10*i+3);
    end
end

