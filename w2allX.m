function xVec = w2allX(w)
    N = (length(w)-8)/10;
    xVec = zeros(N+1,1);
    for i=0:N
        xVec(i+1) = w(10*i+1);
    end
end

