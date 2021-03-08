function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

% horizon length (prediction AND control)
N = param.N;

% inital guess w0
persistent w0
if isempty(w0)
    w0 = param.w_guess(1:10*N+8);
else
    w0(1:end-10) = w0(11:end);
    [~, ~, fref] = getLinearisation(w0(end-17:end-10), w0(end-9:end-8), 10, param.Ts, param.modelDerivative, param.genericA, param.genericB);
    w0(end-7:end) = fref;
end


% objective function (w contains N+1 x vectors and N u vectors)
%objFunc = @(w) objFuncN(w, N);

% linear inequality constraint
[A, b] = inequalityConstraints(N, r, param.tolerances.state(1:8), param.craneParams.r, param.constraints.rect, param.constraints.ellipses, w0);

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
penalties = zeros(8+10*N,1);
for i=1:N
    penalties(10*i-1:10*i) = ones(2,1); 
end
w = quadprog(diag(penalties),zeros(1,length(penalties)),A,b,Aeq,beq);

% extract u from w
u = w(9:10);

end % End of myMPController







%% objective function

