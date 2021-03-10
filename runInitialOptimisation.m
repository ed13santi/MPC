function w = runInitialOptimisation(finalTrgt, x_hat, param)

% horizon length (prediction AND control)
N = param.Tf / param.Ts;

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncInitialGuess(w, N);

% initial guess w0
w0 = [x_hat; zeros(10*N, 1)];
for i=1:N
    w0(10*i+1:10*i+8) = (finalTrgt - x_hat) / N * i + x_hat;
end

% linear inequality constraint
[A, b] = inequalityConstraintsInitialGuess(N, param.craneParams.r, param.constraints.rect);

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = linearEqConstrInitialGuess(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, param.Ts, finalTrgt);

% non-linear constraints
nonlcon = @(w) nonLinearConstraints(true, param.constraints.ellipses, param.modelDerivative, param.Ts, w);

% options
options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);

% optimisation
A = sparse(A);
Aeq = sparse(Aeq);
w = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);
%w(1:8)

end % End of myMPController
