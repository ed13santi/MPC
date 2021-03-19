function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

x_hat = x_hat(1:8);

% keep track of when to run optimisation and when just use previous results
persistent reoptimiseCount
if isempty(reoptimiseCount)
    reoptimiseCount = 1;
else
    if reoptimiseCount < param.optimiseEvery
        reoptimiseCount = reoptimiseCount + 1;
    else 
        reoptimiseCount = 1;
    end
end

% keep track of index of current iteration
persistent iter
if isempty(iter)
    iter = 0;
else
    iter = iter + 1;
end

% declare persisten variables 
persistent prevW
persistent save_u
persistent iA
persistent prevITS


if reoptimiseCount == 1
% horizon length (prediction AND control)
N = max(min(param.N, param.Tf/param.Ts - iter), 5);

% inital guess w0
% persistent w0
% if isempty(w0)
%     w0 = param.wref(1:10*N+8);
% else
%     w0(1:end-10) = w0(11:end);
%     [~, ~, fref] = getLinearisation(w0(end-17:end-10), w0(end-9:end-8), 10, param.Ts, param.modelDerivative, param.genericA, param.genericB);
%     w0(end-7:end) = fref;
% end

% number of slack variables for soft constraints
nSlackVars = param.nSlackVars;

secLen = 10 + nSlackVars;
% w0 = [param.wref(iter*secLen+1:iter*secLen+8); % x
%       param.wref((iter+1)*secLen-1:(iter+N)*secLen+8+nSlackVars)]; % uxXuxXuxXuxXuxX
% w0 = xuxXuxXuxXuxXuxX

% wref = % xXuxXu...xXuxX
if isempty(prevW)
    start = (iter+1)*secLen+9+nSlackVars; % start of u
    fin   = (iter+N+1)*secLen+8+nSlackVars; % end of X ---> uxXuxXuxXuxXuxX
    refTraj = [x_hat(1:8); param.wref(start:fin)]; % xuxXuxXuxXuxXuxX
else
    piece1 = x_hat(1:8); % 
    % prevW = xuxXuxXuxXuxXuxX
    piece2 = prevW(9+secLen*param.optimiseEvery:8+secLen*N); % uxXuxXuxXuxXuxX
    start = (iter+N+1)*secLen-1; % start of u
    fin   = (iter+N+1)*secLen+8+nSlackVars+(param.optimiseEvery-1)*secLen; % end of X 
    piece3 = param.wref(start:fin); % uxXuxXuxXuxXuxX
    refTraj = [piece1; piece2; piece3]; % xuxXuxXuxXuxXuxX
end

% objective function (w contains N+1 x vectors and N u vectors)
%objFunc = @(w) objFuncN(w, N);

% linear inequality constraint
stateTol = param.tolerances.state(1:8);
inputTol = param.tolerances.input(1:2);
ropeLen =  param.craneParams.r;
rectConstr = param.constraints.rect;
ellConstr = param.constraints.ellipses;
n_at_equilibrium = max(0, iter - param.Tf / param.Ts + 1 + N);
extraDistEll = param.extraDistanceEllipses;
extraDistRect = param.extraDistanceRectangles;
[A, b, idxsToShift] = inequalityConstraints(N, r, stateTol, inputTol, ropeLen, rectConstr, ellConstr, refTraj, n_at_equilibrium, extraDistEll, extraDistRect, nSlackVars);

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = getStateSpace(x_hat, refTraj, param.genericA, param.genericB, param.modelDerivative, N, param.Ts, nSlackVars);

% optimisation

% penBlock = [ones(10,1); zeros(10*(param.TsFactor-1),1)];
% n_blocks = N/param.TsFactor;
% penalties = [kron(ones(n_blocks,1), penBlock); ones(8,1)];
xPen = 1;
uPen = 1;
lambdaPen = 1e3;
finLambdaPen = 1e6;
penaltyBlk = [uPen * ones(2,1); xPen * ones(8,1); lambdaPen * ones(nSlackVars,1) ];  %uxX
penalties = [ xPen * ones(8,1); kron(ones(N,1), penaltyBlk) ];  % xuxXuxXuxXuxX
penalties(end-7-nSlackVars:end-nSlackVars) = 10 * xPen * ones(8,1);
%penalties(end-nSlackVars-7:end-nSlackVars) = lambdaPen * ones(8,1);
penalties = [penalties; finLambdaPen * ones(5,1)];
H = diag(penalties);

penaltyBlkf = [uPen * ones(2,1); xPen * ones(8,1); zeros(nSlackVars,1) ]; 
penaltiesf = [ xPen * ones(8,1); kron(ones(N,1), penaltyBlkf) ];
penaltiesf(end-7-nSlackVars:end-nSlackVars) = 10 * xPen * ones(8,1);
%penaltiesf(end-nSlackVars-7:end-nSlackVars) = lambdaPen * ones(8,1);
penaltiesf = [penaltiesf; zeros(5,1)];

Hf = diag(penaltiesf);


%refTraj = param.wref(iter*10+1:(iter+N)*10+8);
f = - Hf * [refTraj; zeros(5,1)];
% size(H)
% size(f)
% size(A)
% size(b)
% size(Aeq)
% size(beq)

% H = sparse(H);
% A = sparse(A);
% Aeq = sparse(Aeq);

% opt = mpcInteriorPointOptions;
% w = mpcInteriorPointSolver(H,f,A,b,Aeq,beq,[refTraj; zeros(5,1)],opt);
if isempty(iA)
    iA = false(size(A,1),1);
end
if isempty(prevITS)
    prevITS = [];
end
iA = [iA(prevITS); false(size(A,1)-length(prevITS),1)];
prevITS = idxsToShift;
opt = mpcActiveSetOptions;
[w,~,iA,~] = mpcActiveSetSolver(H,f,A,b,Aeq,beq,iA,opt);
% opt = optimoptions('quadprog','Algorithm','active-set');
% w = quadprog(H,f,A,b,Aeq,beq,[],[],[refTraj; zeros(5,1)],opt);

prevW = w(1:end-5);

% extract u from w
save_u = [];
for oC=1:param.optimiseEvery
save_u = [save_u, w(9+(oC-1)*secLen:10+(oC-1)*secLen)];
end

end

u = save_u(:, reoptimiseCount);

end % End of myMPController







%% objective function

