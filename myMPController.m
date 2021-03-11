function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

% horizon length (prediction AND control)
N = param.N;

% inital guess w0
% persistent w0
% if isempty(w0)
%     w0 = param.w_guess(1:10*N+8);
% else
%     w0(1:end-10) = w0(11:end);
%     [~, ~, fref] = getLinearisation(w0(end-17:end-10), w0(end-9:end-8), 10, param.Ts, param.modelDerivative, param.genericA, param.genericB);
%     w0(end-7:end) = fref;
% end
extraCopies = 20 / param.Ts + N - (length(param.w_guess) - 8)/10;
wref = [ param.w_guess; kron(ones(extraCopies,1), [0;0;param.w_guess(end-7:end)]) ]; 

persistent iter
if isempty(iter)
    iter = 0;
else
    iter = iter + 1;
end

w0 = wref(iter*10+1:(iter+N)*10+8);


% objective function (w contains N+1 x vectors and N u vectors)
%objFunc = @(w) objFuncN(w, N);

% linear inequality constraint
n_at_equilibrium = 0; %max(0, min(N, iter - param.Tf / param.Ts + 1));
[A, b] = inequalityConstraints(N, r, param.tolerances.state(1:8), param.craneParams.r, param.constraints.rect, param.constraints.ellipses, w0, n_at_equilibrium);

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = getStateSpace(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, param.Ts);

% non-linear constraints
% nonlcon = @(w) nonLinearConstraints(param.Ts, param.craneParams, w);

% options
% options = optimoptions(@fmincon);

% optimisation
A = sparse(A);
Aeq = sparse(Aeq);
% w = fmincon(objFunc,w0,A,b,Aeq,beq);
% H = zeros(size(A,2));
% for i=1:N
%     H(i*13-2:i*13-1,i*13-2:i*13-1) = eye(2);
% end
% H = sparse(H);
% options = optimoptions('fmincon','Algorithm','interior-point');
% w = quadprog(H,zeros(1,size(A,2)),A,b,Aeq,beq);
penalties = ones(8+10*N,1);
H = diag(penalties);

refTraj = wref(iter*10+1:(iter+N)*10+8);
f = - H * refTraj;
w = quadprog(H,f,A,b,Aeq,beq);

% extract u from w
u = w(9:10);

end % End of myMPController







%% objective function

