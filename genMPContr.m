function u = genMPContr(H,G,F,bb,J,L,x,xTarget,m,N,Phi,Gamma)
    linTerm = G * (x - xTarget);
    if size(J,1) == 0
        rightIneqConstr = [];
    else
        rightIneqConstr = bb + J*x + L*xTarget;
    end
    
    persistent u0
    if isempty(u0)
        u0 = zeros(m*N,1);
    end
    diff = length(u0) - m*N;
    if diff ~= 0
        u0 = u0(1+diff:end);
    end
    
    options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
    U = quadprog(H, linTerm, F, rightIneqConstr, [], zeros(0,1), [], [], u0, options);
    u = U(1:2);
    u0 = U;
end



