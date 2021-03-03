function out = objFuncN(w, N)
    out = 0;
    for i = 1:N
        out = out + workOfStep(w(i*13-12:i*13+10));
    end
end

