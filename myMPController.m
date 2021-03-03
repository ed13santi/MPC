function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

% horizon length
N = param.N;

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncN(w, N);

% inital guess w0
persistent w0
if isempty(w0)
    w0 = zeros(13*N+10, 1);
else
    w0(1:13*N) = w0(11:13*N+10);
end

% linear inequality constraint
A = [];
b = [];

% linear equality constraints (currently only equality constraint on x0)
Aeq = leftEquality(N);
beq = rightEquality(N, x_hat(1:8), param.craneParams.r);

% non-linear constraints
nonlcon = @(w) nonLinearConstraints(param.Ts, param.craneParams, w);

% lower and upper bounds
wLen = N*13+10;
lb = lbConstraint(wLen, r(1:8), param.tolerances.state(1:8));
ub = ubConstraint(wLen, r(1:8), param.tolerances.state(1:8));

% options
options = optimoptions(@fmincon);

% optimisation
w = fmincon(objFunc,w0,A,b,Aeq,beq,lb,ub,nonlcon,options);

% extract u from w
u = w(11:12);

end % End of myMPController







%% objective function

