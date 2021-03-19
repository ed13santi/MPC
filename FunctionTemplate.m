function [ param ] = mySetup(shape)
%% Modify the following function for your setup function

% Copy course parameters
param.constraints = shape.constraints;
param.tolerances = shape.tolerances; 
param.eps_r = shape.eps_r;      
param.eps_t = shape.eps_t;      
param.start = shape.start;      
param.target = shape.target;     
param.Wmax = shape.Wmax;   

param.Ts = 0.1; % set sampling period
param.TsFactor = 5; % set sampling frequency reduction factor for initial non-linear optimisation
param.Tf = shape.Tf - param.Ts; % set Tf to be slightly less than required
N_initial = ceil(param.Tf / (param.Ts * param.TsFactor));
param.Ts = param.Tf / (N_initial * param.TsFactor);
param.optimiseEvery = 1;
param.nInitialOpimisations = 2;

param.extraDistanceEllipses = 0.001;
param.extraDistanceRectangles = 0.001;
n_ellipses = size(param.constraints.ellipses, 1) * size(param.constraints.ellipses, 2);
param.nSlackVars = 4 + 2*min(2, n_ellipses);

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% This is a sample way to send reference points
initialState = [shape.start(1); 0; shape.start(2); 0; 0; 0; 0; 0];
targetState  = [param.xTar;     0; param.yTar;     0; 0; 0; 0; 0];

% Horizon length
param.N = 15;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

% Create generic state space that can be used at every time step using the
% current values of x and u
[param.genericA, param.genericB, param.modelDerivative] = obtainJacs(param.craneParams);

%Run initial optimisation over whole length of episode using fully
%non-linear model
param.w_guess = runInitialOptimisation(targetState, initialState, param, param.TsFactor, N_initial);
% convert to xXuxXu...xXuxX format
param.w_guess = convertToIncludeSlackVars(param.w_guess, param.nSlackVars);

segLen = 10+param.nSlackVars;
simulationLength = 20;
extraCopies = ceil(simulationLength / param.Ts) + param.N - (length(param.w_guess) - segLen + 2)/segLen + param.TsFactor;
blk = [0;0;param.w_guess(end-7-param.nSlackVars:end-param.nSlackVars);zeros(param.nSlackVars,1)]; % uxX
repBlk = kron(ones(extraCopies,1), blk); % uxXuxXuxXuxXuxXuxXuxX
param.wref = [ param.w_guess; repBlk ]; % xXuxXu...xXuxX

figure;
plotx = [];
ploty = [];
plotxp = [];
plotyp = [];
for i=1:(length(param.w_guess)-8)/segLen+1
   plotx = [plotx param.w_guess((i-1)*segLen+1)]; 
   ploty = [ploty param.w_guess((i-1)*segLen+3)]; 
   plotxp = [plotxp param.w_guess((i-1)*segLen+1)+param.craneParams.r*sin(param.w_guess((i-1)*segLen+5))]; 
   plotyp = [plotyp param.w_guess((i-1)*segLen+3)+param.craneParams.r*sin(param.w_guess((i-1)*segLen+7))];
end
scatter(plotx, ploty);
hold on;
scatter(plotxp, plotyp);
hold off;



end % End of mySetup

function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

r = [param.xTar; 0; param.yTar; 0; 0; 0; 0; 0];

end % End of myTargetGenerator

function x_hat = myStateEstimator(u, y, param)
%% Modify the following function for your state estimator (if desired)

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
x_hat = zeros(8,1);

% By default, just pass the system measurements through
x_hat( 1:length(y),1 ) = y;

end % End of myStateEstimator

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

function out = objFuncN(w, N)
    penalties = zeros(8+10*N,1);
    for i=1:N
       penalties(10*i-1:10*i) = ones(2,1); 
    end
    out = w' * diag(penalties) * w;
%     vels = zeros(2*(N+1), 1);
%     us = zeros(2*N, 1);
%     for i = 1:N
%         vels(2*i-1:2*i, 1) = w(10*i-8:2:10*i-6);
%         us(2*i-1:2*i, 1) = w(10*i-1:10*i);
%     end
%     vels(2*(N+1)-1:2*(N+1), 1) = w(10*(N+1)-8:2:10*(N+1)-6);
%     
%     vels = vels(3:end,1) + vels(1:end-2,1);
%     prods = us .* vels;
%     maxed = max(prods,[],2);
%     out = sum(maxed);
end


%% linear inequality constraints


function [A, b, indexesToShift] = inequalityConstraints(N, r, tolState, tolInput, ropeLen, rectConstraints, ellipses, w, n_final, extraDistanceEllipses, extraDistRect, nSlackVars)
    secLen = 10 + nSlackVars;
    
    indexesToShift = [];

    if n_final < N + 1
        A = zeros(0, 8+secLen*N+5);
        b = zeros(0, 1);
    else
        A = [ zeros(4,8), [eye(2); -eye(2)], zeros(4,secLen*N-2+5) ];
        b = [ tolInput; tolInput ];
    end
    
    for i=1:N
        x = w(10+(i-1)*secLen+1);
        y = w(10+(i-1)*secLen+3);
        theta = w(10+(i-1)*secLen+5);
        phi = w(10+(i-1)*secLen+7);
        
        
        % input physical limits
        [A1, b1] = physicalLimsWithSlack(nSlackVars);
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen, theta, phi, extraDistRect, nSlackVars);
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ropeLen, ellipses, x, y, theta, phi, extraDistanceEllipses, nSlackVars);
        
        A_tmp = [A1;A2;A3];
        %                              xuxX             uxX      uxXuxX
        A_tmp2 = [zeros(size(A_tmp,1), 8+(i-1)*secLen), A_tmp, zeros(size(A_tmp,1), secLen*(N-i)+5)];
        A = [A; A_tmp2];
        b = [b;b1;b2;b3];
        
        if i>=2
            indexesToShift = [indexesToShift, size(A,1)+1-size(A_tmp2,1):size(A,1)];
        end
        
        if N - i < n_final
            % final state/input constraint
            [A4, b4] = finalRows(r, tolState, tolInput, i, N, nSlackVars);
            A = [A; A4];
            b = [b; b4];
            if i == 1
                indexesToShift = [indexesToShift, size(A,1)-3:size(A,1)];
            end
            if i>1
                indexesToShift = [indexesToShift, size(A,1)+1-size(A4,1):size(A,1)];
            end
        end
    end
    
%     if n_final > 0
%         % final state/input constraint
%         [A_tmp, b_tmp] = finalRows(r, tolState, tolInput, N, N, nSlackVars);
%         A = [A; A_tmp];
%         b = [b; b_tmp];
%     end
    
    % lambda > 0
    A = [ A; [zeros(5,size(A,2)-5), -eye(5)] ];
    b = [ b; zeros(5,1) ];
end

function out = linePars(a,b) 
    if a(1) == b(1)
    out = [-1, 0, -a(1)]; % -x + 0y = -x_a
    else
    tmp = polyfit([a(1) b(1)],[a(2) b(2)],1); % tmp = [a b]  where y=ax+b
    out = [-tmp(1) 1 tmp(2)]; %  
    out = out/(abs(tmp(1))+1);
    end
end

function [mat, ch, cl] = rectCon(rect)
    A = rect(1,:);
    B = rect(2,:);
    C = rect(3,:);
    D = rect(4,:);
    
    a1 = linePars(A,D);
    a2 = linePars(A,B);
    a3 = linePars(B,C);
    a4 = linePars(D,C);
    
    mat  = [ a1(1) , a1(2);
             a2(1) , a2(2) ];
         
    ch = [ max(a1(3),a3(3)); max(a2(3),a4(3)) ];
    cl = [ min(a1(3),a3(3)); min(a2(3),a4(3)) ];
end

function [ARows, bRows] = physicalLims()
    ARows = zeros(4,10);
    ARows(1,1) =  1; % u < 1
    ARows(2,1) = -1; % u > 1
    ARows(3,2) =  1; % u < 1
    ARows(4,2) = -1; % u > 1
    bRows = [1;1;1;1];
end

function [ARows, bRows] = physicalLimsWithSlack(nSlackVars)
    ARows = zeros(4,10+nSlackVars);
    ARows(1,1) =  1; % u < 1
    ARows(2,1) = -1; % u > 1
    ARows(3,2) =  1; % u < 1
    ARows(4,2) = -1; % u > 1
    bRows = [1;1;1;1];
end

function [ARows, bRows] = rectLimsRows(rectConstraints, ropeLen, thetag, phig, extraDistRect, nSlackVars)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    D = [ 0 0 DRect(1,1) 0 DRect(1,2) 0 0             0 0             0; 
          0 0 DRect(2,1) 0 DRect(2,2) 0 0             0 0             0; 
          0 0 DRect(1,1) 0 DRect(1,2) 0 bx*DRect(1,1) 0 by*DRect(1,2) 0;
          0 0 DRect(2,1) 0 DRect(2,2) 0 by*DRect(2,1) 0 by*DRect(2,2) 0 ];
    ARows = [ [ D,                    zeros(4,max(0, nSlackVars)) ]; 
              [ D,           -eye(4), zeros(4,max(0, nSlackVars-4)) ]; 
              [ zeros(4,10), -eye(4), zeros(4,max(0, nSlackVars-4)) ];
              [ -D,          	      zeros(4,max(0, nSlackVars)) ];
              [ -D,          -eye(4), zeros(4,max(0, nSlackVars-4)) ]; 
              [ zeros(4,10), -eye(4), zeros(4,max(0, nSlackVars-4)) ] ];
    offset = [ - ax * DRect(1,1) - ay * DRect(1,2);
               - ax * DRect(2,1) - ay * DRect(2,2) ];
    bRows = [ chRect; 
              chRect + offset; 
              chRect - extraDistRect * ones(2,1); 
              chRect + offset - extraDistRect * ones(2,1); 
              zeros(4,1);
              - clRect;
              - clRect - offset;
              - clRect - extraDistRect * ones(2,1);
              - clRect - offset - extraDistRect * ones(2,1);
              zeros(4,1) ];
end

function [ARows, bRows] = ellipseLimsRows(ropeLen, ellipses, xg, yg, thetag, phig, extraDistance, nSlackVars)
    ellList = [];
    ellVals = [];
    ellObjVals = [];
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            ellList = [ ellList; ell ];
            % build list of the values of ellipseEval for cart
            ellVals = [ ellVals; distanceToEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b) ];
            % build list of the values of ellipseEval for object
            ellObjVals = [ ellObjVals; distanceToEllipse(xg+ropeLen*sin(thetag), yg+ropeLen*sin(phig), ell.xc, ell.yc, ell.a, ell.b) ];
        end
    end
    [~,indexesCart] = sort(ellVals, 'ascend'); % indexes from closest to cart
    [~,indexesObj] = sort(ellObjVals, 'ascend'); % indexes from closest to object
    
    ARows = zeros(0,10+nSlackVars);
    bRows = zeros(0,1);
    
    ellipseSlackVars = nSlackVars - 4;
    
    % constraints of cart
    for i=1:ellipseSlackVars/2
        ell = ellList(indexesCart(i));
        [ARow1, bRow1] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b, extraDistance, ellipseSlackVars, i);
        ARows = [ARows; ARow1];
        bRows = [bRows; bRow1];
    end 
    %constraints of object
    for i=1:ellipseSlackVars/2
        ell = ellList(indexesObj(i));
        [ARow2, bRow2] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, ell.xc, ell.yc, ell.a, ell.b, extraDistance, ellipseSlackVars, i);
        ARows = [ARows; ARow2];
        bRows = [bRows; bRow2];
    end 
end

function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b, extraDist, ellSlackVars, ellIndex)
    % hard 
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ARowH = [ 0, 0, -beta, 0, -gamma, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    ARowH = [ ARowH, slackVars ];
    bRowH = alpha - beta * xg - gamma * yg;

    % soft
    a_new = a + 2*extraDist*abs(a)/(abs(a)+abs(b));
    b_new = b + 2*extraDist*abs(b)/(abs(a)+abs(b)); % pad around ellipse for robustness
    alpha = ellipseEval(xg, yg, xc, yc, a_new, b_new);
    beta = 2 * (xg - xc) / (a_new^2);
    gamma = 2 *(yg - yc) / (b_new^2);
    divBy = abs(beta) + abs(gamma);
    ARowS = [ 0, 0, -beta/divBy, 0, -gamma/divBy, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];
    slackVars(:, 2*ellIndex-1) = -1;
    ARowS = [ ARowS, slackVars ];
    ARowS = [ ARowS; [zeros(1, 14), slackVars] ];
    bRowS = [ (alpha - beta * xg - gamma * yg)/divBy;
              0 ];
         
    ARow = [ARowH;ARowS];
    bRow = [bRowH;bRowS];
end

function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b, extraDist, ellSlackVars, ellIndex)
    % hard
    xg_p = xg + ropeLen * sin(thetag); 
    yg_p = yg + ropeLen * sin(phig); 
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a, b);
    beta =  2 * (xg_p - xc) / (a^2);
    gamma = 2 * (yg_p - yc) / (b^2);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARowH = [ 0, 0, -beta, 0, -gamma, 0, -beta*bx, 0, -gamma*by, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    ARowH = [ ARowH, slackVars ];
    bRowH = alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay;
    
    % soft
    a_new = a + 2*extraDist*abs(a)/(abs(a)+abs(b));
    b_new = b + 2*extraDist*abs(b)/(abs(a)+abs(b)); % pad around ellipse for robustness
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a_new, b_new);
    beta =  2 * (xg_p - xc) / (a_new^2);
    gamma = 2 * (yg_p - yc) / (b_new^2);
    divBy = abs(beta*(1+bx)) + abs(gamma*(1+by));
    ARowS = [ 0, 0, -beta/divBy, 0, -gamma/divBy, 0, -beta*bx/divBy, 0, -gamma*by/divBy, 0, 0, 0, 0, 0 ];
    slackVars(2*ellIndex) = -1;
    ARowS = [ ARowS, slackVars ];
    ARowS = [ ARowS; [zeros(1, 14), slackVars] ];
    bRowS = [ (alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay)/divBy;
             0 ];
         
    ARow = [ARowH;ARowS];
    bRow = [bRowH;bRowS];
end

function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N, nSlackVars)
    segLen = 10 + nSlackVars;
    
    slacks = [ -1 0 0 0 0;
               0 -1 0 0 0;
               -1 0 0 0 0;
               0 -1 0 0 0;
               0 0 -1 0 0;
               0 0 0 -1 0;
               0 0 -1 0 0;
               0 0 0 -1 0];
    slacksU = [ 0 0 0 0 -1;
                0 0 0 0 -1];
               
    if i == N
        
        %         xuxXuxXuxXu                 x                  X
        ARows = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, nSlackVars), [slacks;slacks] ]; 
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        %          xuxXuxXuxXu                 x                  XuxXuxX
        ARows1 = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, segLen*(N-i)+nSlackVars),[slacks;slacks] ];
        %          xuxXuxXuxXuxX                         u                  xXuxX
        ARows2 = [ zeros(4, 18+nSlackVars+segLen*(i-1)), [eye(2); -eye(2)], zeros(4, segLen*(N-i)-2),[slacksU;slacksU] ];
        ARows = [ARows1; ARows2];
        
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2;
                  tolerancesInput;
                  tolerancesInput ];
    end
end




%% linear equality constraints
function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, Ts, nSlackVars)
    secLen = 10 + nSlackVars;
    xus = [x0; w0(9:end-8)]; % xuxXuxXuxXuxX
    Aeq = zeros(8+8*N, length(w0)+5); 
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
    
    x = x0;
    u = w0(9:10);
    [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
    Aeq(9:16, 1:8) = A; 
    Aeq(9:16, 9:10) = B;
    Aeq(9:16, 11:18) = - eye(8);
    beq(9:16) = A*x + B*u - fref;
       
    for i=2:N
       x = xus(10+(i-2)*secLen+1:10+(i-2)*secLen+8); 
       u = xus(10+(i-2)*secLen+9+nSlackVars:10+(i-2)*secLen+10+nSlackVars);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, 10+(i-2)*secLen+1:10+(i-2)*secLen+8) = A; 
       Aeq(8+i*8-7:8+i*8, 10+(i-2)*secLen+9+nSlackVars:10+(i-2)*secLen+10+nSlackVars) = B;
       Aeq(8+i*8-7:8+i*8, 10+(i-1)*secLen+1:10+(i-1)*secLen+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end
end


%% model linearisation
function [Fs, Fv, der] = obtainJacs(cP)

    syms dx x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 ux uy uz
    g  = 9.81;
    m  = cP.m;
    Tl = cP.Tl;
    Ty = cP.Ty;
    Tx = cP.Tx;
    M  = cP.M;
    MR = cP.MR;
    Vx = cP.Vx;
    Vy = cP.Vy;
    Vl = cP.Vl;
    r  = cP.r;
    
    dx = sym(zeros(8, 1));

    dx(1, 1)  = x2;
    dx(2, 1)  = (cos(x5) .* cos(x7) .^ 2 .* sin(x5) .* g .* m + sin(x5) .* (Tl .* x10 - Vl * uz) .* cos(x7) - Tx .* x2 + Vx * ux) ./ (M + MR);
    dx(3, 1)  = x4;
    dx(4, 1)  = ((m .* g .* cos(x5) .* cos(x7) + Tl .* x10 - Vl * uz) .* sin(x7) - Ty .* x4 + Vy * uy) ./ M;
    dx(5, 1)  = x6;  
    dx(6, 1)  = (-cos(x5) .^ 2 .* cos(x7) .^ 2 .* sin(x5) .* g .* m + (-sin(x5) .* (Tl .* x10 - Vl * uz) .* cos(x5) - 0.2e1 .* x6 .* x10 .* (M + MR)) .* cos(x7) + (Tx .* x2 - Vx * ux) .* cos(x5) + 0.2e1 .* (M + MR) .* (sin(x7) .* x6 .* x8 .* x9 - sin(x5) .* g ./ 0.2e1)) ./ x9 ./ cos(x7) ./ (M + MR);
    dx(7, 1)  = x8;
    dx(8, 1)  = (-cos(x5) .* g .* sin(x7) .* m .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 2 + ((-M .* (Tl .* x10 - Vl * uz) .* cos(x5) .^ 2 - M .^ 2 .* x6 .^ 2 .* x9 - x6 .^ 2 .* x9 .* MR .* M - MR .* (Tl .* x10 - Vl * uz)) .* sin(x7) - (-Ty .* x4 + Vy * uy) .* (M + MR)) .* cos(x7) - 0.2e1 .* ((g .* (M + MR) .* cos(x5) ./ 0.2e1 + sin(x5) .* (Tx .* x2 - Vx * ux) ./ 0.2e1) .* sin(x7) + x10 .* x8 .* (M + MR)) .* M) ./ x9 ./ M ./ (M + MR);
    
    dx = subs(dx, [x9 x10 uz], [r 0 0]);
    
    jacA = jacobian(dx, [x1 x2 x3 x4 x5 x6 x7 x8]);
    jacB = jacobian(dx, [ux uy]);

    Fs  = matlabFunction(jacA, 'Vars', [x1, x2, x3, x4, x5, x6, x7, x8, ux, uy]);
    Fv  = matlabFunction(jacB, 'Vars', [x1, x2, x3, x4, x5, x6, x7, x8, ux, uy]);
    der = matlabFunction(dx  , 'Vars', [x1, x2, x3, x4, x5, x6, x7, x8, ux, uy]);
    
    Fs  = @(w)  Fs(w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10));
    Fv  = @(w)  Fv(w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10));
    der = @(w) der(w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10));
end

function [A, B, x_next] = getLinearisation(x, u, Ns, Ts, F, Fs, Fv) %using Euler's method
    x_next = x;
    A = eye(8);
    B = zeros(8,2);
        
    Ti = Ts/Ns;
    for i=1:Ns
        x_next = x_next + Ti * F([x_next;u]);
        tmp = (eye(8) + Ti * Fs([x_next;u])) * [A, B] + [zeros(8), Ti * Fv([x_next;u])];
        A = tmp(:,1:8);
        B = tmp(:,9:10);
    end
end



%% non-linear constraints for initial guess

function x = w2x(w)
    N = (length(w)-8)/10;
    x = zeros(N*8, 1);
    for i=1:N
        x(i*8-7:i*8) = w(i*10+1:i*10+8);
    end
end

function x = doStep(w, dt, Ts) 
    N = (length(w)-8)/10;
    x = zeros(8*N,1);
    for i = 1:N
        x_before = w(i*10-9:i*10-2);
        odeFun = @(t,y) dt([y; w(i*10-1:i*10)]);
        [~, y] = ode45(odeFun, [0 Ts], x_before);
        x(i*8-7:i*8) = y(end,:);
    end
end

function out = ellipseEval(x, y, xc, yc, a, b)
    out = ((x - xc)/a)^2 + ((y - yc)/b)^2 - 1;
end

function [c, ceq] = nonLinearConstraints(rectConstraints, ropeLen, ellConstr, ellipses, dt, Ts, w)
    % inequality constraints for ellipses
    if ellConstr == true
        N = (length(w)-8)/10;
        c = [];
        for i=1:N-1
            x = w(10*i+1);
            y = w(10*i+3);
            angle_x = w(10*i+5);
            angle_y = w(10*i+7);
            x_p = x + ropeLen * sin(angle_x);
            y_p = y + ropeLen * sin(angle_y);
            for j=1:size(ellipses,1)
                for z=1:size(ellipses,2)
                    ell = ellipses{j,z};
                    c = [c; - ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b)];
                    c = [c; - ellipseEval(x_p, y_p, ell.xc, ell.yc, ell.a, ell.b)];
                end
            end
        end
    else
        c = [];
    end
    
    % object rectangular constraints
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    for i=1:N-1
        x = w(10*i+1);
        y = w(10*i+3);
        angle_x = w(10*i+5);
        angle_y = w(10*i+7);
        x_p = x + ropeLen * sin(angle_x);
        y_p = y + ropeLen * sin(angle_y);
        D = [ DRect(1,1) DRect(1,2); 
              DRect(2,1) DRect(2,2)];
        ARows = [D; -D];
        bRows = [ chRect; -clRect ];
        tmpRow = ARows * [x_p; y_p] - bRows;
        c = [c; tmpRow];
    end
    
    % equality constraints due to dynamics of system
%     ceq = w2x(w) - doStep(w, dt, Ts); 
    ceq = [];
end


%% run initial optimisation

function out = objFuncInitialGuess(w, N)
    vels = zeros(2*(N+1), 1);
    us = zeros(2*N, 1);
    for i = 1:N
        vels(2*i-1:2*i, 1) = w(10*i-8:2:10*i-6);
        us(2*i-1:2*i, 1) = w(10*i-1:10*i);
    end
    vels(2*(N+1)-1:2*(N+1), 1) = w(10*(N+1)-8:2:10*(N+1)-6);
    
    vels = vels(3:end,1) + vels(1:end-2,1);
    prods = us .* vels;
    maxed = max(prods,[],2);
    out = sum(maxed);
end

function w = runInitialOptimisation(finalTrgt, x_hat, param, factorTs, N)

% horizon length (prediction AND control)
Ts = param.Ts * factorTs; % initial guess to uses longer Ts to reduce set-up time

% find approximate path (no inputs)
% xs = findPath(finalTrgt, x_hat, N, param);
A = param.constraints.rect(1,:);
B = param.constraints.rect(2,:);
C = param.constraints.rect(3,:);
D = param.constraints.rect(4,:);
xs = makePath(A',B',C',D',0.005,[x_hat(1); x_hat(3)],[finalTrgt(1); finalTrgt(3)],param.constraints.rect,param.constraints.ellipses);
xs = changeToCorrectLength(xs, N);
pathPlanningDone = "PATH PLANNING COMPLETED"
figure;
% scatter(xs(1:2:end),xs(2:2:end));
scatter(xs(1,:),xs(2,:));

% initial guess w0
w0 = plannedPath2wrefInitial(xs);

% % objective function (w contains N+1 x vectors and N u vectors)
% objFunc = @(w) objFuncPath(w, w0, N);
% 
% % linear inequality constraint
% [A, b] = inequalityConstraintsInitialGuess(N, param.craneParams.r, param.constraints.rect, finalTrgt, param.tolerances.state(1:8), param.tolerances.input(1:2));
% 
% % linear equality constraints (currently only equality constraint on x0)
% [Aeq, beq] = linearEqConstrInitialGuess(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, Ts, finalTrgt);
% 
% % non-linear constraints
% nonlcon = @(w) nonLinearConstraints(param.constraints.rect, param.craneParams.r, true, param.constraints.ellipses, param.modelDerivative, Ts, w);
% 
% % options
% options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);
% 
% % optimisation
% A = sparse(A);
% Aeq = sparse(Aeq);
% w0 = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);
    
    
% w0 = [x_hat; zeros(10*N, 1)];
% for i=1:N
%     w0(10*i+1:10*i+8) = (finalTrgt - x_hat) / N * i + x_hat;
% end

    
    

% objective function (w contains N+1 x vectors and N u vectors)
objFunc = @(w) objFuncInitialGuess(w, N);

for relinearisation=1:param.nInitialOpimisations % rerun multiple times using better linearisation

    % linear inequality constraint
    [A, b] = inequalityConstraintsInitialGuess(N, param.craneParams.r, param.constraints.rect, finalTrgt, param.tolerances.state(1:8), param.tolerances.input(1:2));

    % linear equality constraints (currently only equality constraint on x0)
    [Aeq, beq] = linearEqConstrInitialGuess(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, Ts, finalTrgt);

    % non-linear constraints
    nonlcon = @(w) nonLinearConstraints(param.constraints.rect, param.craneParams.r, true, param.constraints.ellipses, param.modelDerivative, Ts, w);

    % options
    options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);

    % optimisation
    A = sparse(A);
    Aeq = sparse(Aeq);
    w0 = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);
end

% interpolate 
w = w0(1:end-10);
w = interpolate(w, factorTs);

end 

function [Aeq, beq] = linearEqConstrInitialGuess(x0, w0, genA, genB, der, N, Ts, r)
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8) = x0;
    
    % linear model constraints
    xus = [x0; w0(9:end-8)];
       
    for i=1:N
       x = xus(i*10-9:i*10-2);
       u = xus(i*10-1:i*10);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, i*10-9:i*10-2) = A; 
       Aeq(8+i*8-7:8+i*8, i*10-1:i*10) = B;
       Aeq(8+i*8-7:8+i*8, i*10+1:i*10+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end

    % final position constraint
    [A_tmp, b_tmp] = finalPositionRowsInitGuess(r, N);
    Aeq = [Aeq; A_tmp];
    beq = [beq; b_tmp];
    
    
    
end

function [A, b] = inequalityConstraintsInitialGuess(N, ropeLen, rectConstraints, r, stateTol, inputTol)
    A = zeros(0, 8+10*N);
    b = zeros(0, 1);
    
    for i=1:N
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRowsInitialGuess(rectConstraints);
        
        % combine matrices
        A_tmp = [A1;A2];
        A_tmp2 = [zeros(size(A_tmp,1), i*10-2), A_tmp, zeros(size(A_tmp,1), 10*N-i*10)];
        A = [A; A_tmp2];
        b = [b; b1;b2];
    end
end

function [ARows, bRows] = finalPositionRowsInitGuess(r, N)
    ARows = [ zeros(8, 10*N), eye(8) ];
    bRows = r;
end

function w_out = interpolate(w, factor) 
    w = [w; 0; 0];
    intStep = 1/factor;
    N = length(w)/10;
    w = reshape(w,[10,N]);
    w_out = [];
    for i=1:10
        interpolatedVec = interp1(1:N, w(i,:), 1:intStep:N,'linear');
        w_out = [ w_out; interpolatedVec ];
    end
    w_out = reshape(w_out,[],1) ;
    w_out = w_out(1:end-2);
end

function [ARows, bRows] = rectLimsRowsInitialGuess(rectConstraints)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    D = [ 0 0 DRect(1,1) 0 DRect(1,2) 0 0 0 0 0; 
          0 0 DRect(2,1) 0 DRect(2,2) 0 0 0 0 0 ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
end

 

%% Find path

function path = makePath(A,B,C,D,h,start,target,rectConstr,ellConstr) %MAKE SURE START AND TARGET ARE 2 ELEMENT VECTORS
    coords = [A,B,C,D];
    minx = min(coords(1,:));
    miny = min(coords(2,:));
    maxx = max(coords(1,:));
    maxy = max(coords(2,:));
    

    found = false;
    while not(found)

        nx = ceil((maxx-minx)/h);
        ny = ceil((maxy-miny)/h);

        outRect = zeros(nx, ny);
        corner = [minx; miny];

        start_ind = [floor((start(1)-minx)/h)+1; floor((start(2)-miny)/h)+1];
        trgt_ind = [floor((target(1)-minx)/h)+1; floor((target(2)-miny)/h)+1];

        for i=1:size(outRect,1)
            for j=1:size(outRect,2)
                x = minx + (i-1) * h + h/2;
                y = miny + (j-1) * h + h/2;
                violate1 = violateRectConstr(rectConstr, x, y);
                violate2 = violateEllConstr(ellConstr, x, y);
                if violate1 | violate2
                    outRect(i,j) = -1;
                end
            end
        end

        outRect(start_ind(1),start_ind(2)) = 1;

        maxIter = 50000;
        iter = 0;
        while not(found) & iter < maxIter
            [outRect, found] = runIterationPathFinding(outRect, trgt_ind);
            iter = iter + 1;
        end

        if not(found)
           fprintf("Path Finding failed, MaxIter reached"); 
           h = h/2; % decrease step size
        end
    end
       
    path = rect2path(outRect, trgt_ind, minx, miny, h);
end

function path = rect2path(rect, trgtInd, minx, miny, h)
    padded  = [zeros(size(rect,1),1), rect, zeros(size(rect,1),1)];
    padded  = [zeros(1,size(padded,2)); padded; zeros(1,size(padded,2))];

    current = trgtInd + [1;1];
    path = [trgtInd];
    finished = false;
    while not(finished)
        currVal = padded(current(1),current(2));
        next = [-1;-1];
        if padded(current(1)+1,current(2)-1) == currVal - 1
            next = current + [1;-1];
        end
        if padded(current(1)+1,current(2)) == currVal - 1
            next = current + [1;0];
        end
        if padded(current(1)+1,current(2)+1) == currVal - 1
            next = current + [1;1];
        end
        if padded(current(1)-1,current(2)-1) == currVal - 1
            next = current + [-1;-1];
        end
        if padded(current(1)-1,current(2)) == currVal - 1
            next = current + [-1;0];
        end
        if padded(current(1)-1,current(2)+1) == currVal - 1
            next = current + [-1;1];
        end
        if padded(current(1),current(2)-1) == currVal - 1
            next = current + [0;-1];
        end
        if padded(current(1),current(2)+1) == currVal - 1
            next = current + [0;1];
        end
        
        path = [path, next - [1;1]];
        if currVal == 2
            finished = true;
        end
        current = next;
    end
    
    for i=1:size(path,2)
        path(1,i) =  minx + (path(1,i)-1) * h;
        path(2,i) =  miny + (path(2,i)-1) * h;
    end
    
    path = flip(path,2);
end

function p = changeToCorrectLength(path, N)
    N_path = size(path,2) - 1;
    intStep = N_path / N;
    p(1,:) = interp1(1:N_path+1, path(1,:), 1:intStep:N_path+1,'linear');
    p(2,:) = interp1(1:N_path+1, path(2,:), 1:intStep:N_path+1,'linear');
end

function [outRect, found] = runIterationPathFinding(outRect, trgt_ind)
    padded  = [zeros(size(outRect,1),1), outRect, zeros(size(outRect,1),1)];
    padded  = [zeros(1,size(padded,2)); padded; zeros(1,size(padded,2))];
    for i=2:size(outRect,1)+1
        for j=2:size(outRect,2)+1
            if padded(i,j) == 0
                neigh = 0;
                neigh = max(padded(i  ,j-1),neigh);
                neigh = max(padded(i  ,j+1),neigh);
                neigh = max(padded(i-1,j-1),neigh);
                neigh = max(padded(i-1,j+1),neigh);
                neigh = max(padded(i-1,j  ),neigh);
                neigh = max(padded(i+1,j-1),neigh);
                neigh = max(padded(i+1,j+1),neigh);
                neigh = max(padded(i+1,j  ),neigh);
                if neigh > 0
                    outRect(i-1,j-1) = neigh + 1;
                end
            end
        end
    end
    found = (outRect(trgt_ind(1),trgt_ind(2)) > 0);
end

function violate = violateRectConstr(rectConstr, x, y)
    [DRect,chRect,clRect] = rectCon(rectConstr);
    D = [ DRect(1,1) DRect(1,2); 
          DRect(2,1) DRect(2,2) ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
    vect = ARows * [x;y] - bRows;
    violate = any(vect > 0);
end

function violate = violateEllConstr(ellipses, x, y)
    violate = false;
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            if ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b) <= 0
                violate = true;
            end
        end
    end
end

function wrefInitial = plannedPath2wrefInitial(path)
    wrefInitial = []
    for i=1:size(path,2)-1
        wrefInitial = [wrefInitial; path(1,i); 0; path(2,i); 0; 0; 0; 0; 0; 0; 0];
    end
    wrefInitial = [wrefInitial; path(1,end); 0; path(2,end); 0; 0; 0; 0; 0];
end

function out = objFuncPath(w, w0, N)
    w_r = [];
    w0_r = [];
    for i=1:N+1
        w_r = [w_r; w(10*i-9); w(10*i-7)];
        w0_r = [w0_r; w0(10*i-9); w0(10*i-7)];
    end
    diff = w_r - w0_r;
    out = diff' * diff;
end


%% manipulate vectors

% convert from xuxu...xux format
% convert to xXuxXu...xXuxX format
function long = convertToIncludeSlackVars(short, nSlackVars)
    short = [0;0;short];  % 00xuxuxuxuxux
    resized = reshape(short,10,[]); % 00x ux ux ux ux
    resized = [resized; zeros(nSlackVars, size(resized,2))]; % 00xX uxX uxX uxX
    long = reshape(resized, size(resized,1)*size(resized,2), 1); % 00xXuxXuxXuxX
    long = long(3:end); % xXuxXuxXuxX
end


function dist = distanceToEllipse(x,y,xc,yc,a,b)
    beta = 1 / (((x-xc)/a)^2 + ((y-yc)/b)^2);
    distToCentre = sqrt((x-xc)^2 + (y-yc)^2);
    dist = distToCentre * (1-sqrt(beta)); 
end
