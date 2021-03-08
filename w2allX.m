function xVec = w2allX(w)
    N = (length(w)-8)/10;
    xVec = zeros(N,1);
    for i=1:N
        xVec(i) = w(10*i+1);
    end
end

