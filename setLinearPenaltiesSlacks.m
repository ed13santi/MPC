function f = setLinearPenaltiesSlacks(f, N, nSlackVars, lambdaPen)
    secLen = 10 + nSlackVars;
    for i=1:N
        f(23+secLen*(i-1):18+nSlackVars+secLen*(i-1)) = lambdaPen * ones(nSlackVars-4,1);
    end
end
