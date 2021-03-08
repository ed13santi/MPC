function w = runInitialOptimisation(finalTrgt, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

% horizon length (prediction AND control)
N = param.Tf / param.Ts;

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncInitialGuess(w, N);

% inital guess w0
w0 = zeros(10*N+8, 1);

% linear inequality constraint
[A, b] = inequalityConstraintsInitialGuess(N, finalTrgt, param.tolerances.state(1:8), param.craneParams.r, param.constraints.rect, param.constraints.ellipses, w0);

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = linearEqConstrInitialGuess(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, param.Ts);

% non-linear constraints
nonlcon = @(w) nonLinearConstraints(param.constraints.ellipses, param.modelDerivative, param.Ts, w);

% options
options = optimoptions(@fmincon);

% optimisation
A = sparse(A);
Aeq = sparse(Aeq);
w = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);

end % End of myMPController

