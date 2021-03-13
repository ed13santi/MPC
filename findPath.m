function xs = findPath(finalTrgt, x_hat, N, param)
    finalTrgt = [finalTrgt(1); finalTrgt(3)];
    x_hat = [x_hat(1); x_hat(3)];
    % initial guess x0
    x0 = x_hat;
    for i=1:N
        x0(2*i+1:2*i+2) = (finalTrgt - x_hat) / N * i + x_hat;
    end
    
    % objective function minimises distance between consecutive xs
    objFunc = @(x) pathObjFunc(x, N);
    
    % linear inequality constraint
    [A, b] = inequalityConstraintsPath(N, param.constraints.rect);

    % linear equality constraints 
    [Aeq, beq] = linearEqConstrPath(x_hat, finalTrgt, N);

    % non-linear constraints
    nonlcon = @(x) nonLinearConstraintsPath(true, param.constraints.ellipses, x, N);

    % options
    options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);
% 
%     size(x0)
%     size(A)
%     size(b)
%     size(Aeq)
%     size(beq)
    
    % optimisation
    A = sparse(A);
    Aeq = sparse(Aeq);
    xs = fmincon(objFunc,x0,A,b,Aeq,beq,[],[],nonlcon,options);
end

