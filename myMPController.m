function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

% horizon length (prediction AND control)
N = param.N;

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncN(w, N);

% inital guess w0
persistent w0
if isempty(w0)
    w0 = zeros(13*N+10, 1);
    for i=9:13:13*N+10
       w0(i,1) = param.craneParams.r; 
    end
else
    w0(1:13*N-3) = w0(14:13*N+10);
end

% linear inequality constraint
[A, b] = inequalityConstraints(N, param.craneParams.r, param.tolerances.state(1:8));

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = getStateSpace(x_hat, w0, param.genericA, param.genericB, N, param.craneParams.r, param.Ts);

% non-linear constraints
% nonlcon = @(w) nonLinearConstraints(param.Ts, param.craneParams, w);

% options
% options = optimoptions(@fmincon);

% optimisation
w = fmincon(objFunc,w0,A,b,Aeq,beq);

% extract u from w
u = w(11:12);

end % End of myMPController







%% objective function

