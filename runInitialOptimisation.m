function w = runInitialOptimisation(finalTrgt, x_hat, param, factorTs)

% horizon length (prediction AND control)
Ts = param.Ts * factorTs; % initial guess to uses longer Ts to reduce set-up time
N = param.Tf / Ts - 1;

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncInitialGuess(w, N);

% initial guess w0
w0 = [x_hat; zeros(10*N, 1)];
for i=1:N
    w0(10*i+1:10*i+8) = (finalTrgt - x_hat) / N * i + x_hat;
end

% linear inequality constraint
[A, b] = inequalityConstraintsInitialGuess(N, param.craneParams.r, param.constraints.rect, finalTrgt, param.tolerances.state(1:8), param.tolerances.input(1:2));

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = linearEqConstrInitialGuess(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, Ts, finalTrgt);

% non-linear constraints
nonlcon = @(w) nonLinearConstraints(true, param.constraints.ellipses, param.modelDerivative, Ts, w);

% options
options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);

% optimisation
A = sparse(A);
Aeq = sparse(Aeq);
w = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);

% interpolate 
w = w(1:end-10);
w = interpolate(w, factorTs);

end % End of myMPController

