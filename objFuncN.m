function out = objFuncN(w, N)
    out = 0;
    for i = 1:13:N*13
        out = out + workOfStep(w(i:i+22));
    end
end

