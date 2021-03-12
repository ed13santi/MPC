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

param.Ts = 0.05; % set sampling period
param.TsFactor = 5; % set sampling frequency reduction factor for initial non-linear optimisation
param.Tf = shape.Tf - param.Ts * param.TsFactor; % set Tf to be slightly less than required


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
param.w_guess = runInitialOptimisation(targetState, initialState, param, param.TsFactor);

figure;
plotx = [];
ploty = [];
plotxp = [];
plotyp = [];
for i=1:(length(param.w_guess)-8)/10+1
   plotx = [plotx param.w_guess(i*10-9)]; 
   ploty = [ploty param.w_guess(i*10-7)]; 
   plotxp = [plotxp param.w_guess(i*10-9)+param.craneParams.r*sin(param.w_guess(i*10-5))]; 
   plotyp = [plotyp param.w_guess(i*10-7)+param.craneParams.r*sin(param.w_guess(i*10-3))];
end
scatter(plotx, ploty);
hold on;
scatter(plotxp, plotyp);
hold off;

extraCopies = 20 / param.Ts + param.N - (length(param.w_guess) - 8)/10 + param.TsFactor;
param.wref = [ param.w_guess; kron(ones(extraCopies,1), [0;0;param.w_guess(end-7:end)]) ]; 

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

% horizon length (prediction AND control)
N = param.N;

% inital guess w0
% persistent w0
% if isempty(w0)
%     w0 = param.wref(1:10*N+8);
% else
%     w0(1:end-10) = w0(11:end);
%     [~, ~, fref] = getLinearisation(w0(end-17:end-10), w0(end-9:end-8), 10, param.Ts, param.modelDerivative, param.genericA, param.genericB);
%     w0(end-7:end) = fref;
% end



persistent iter
if isempty(iter)
    iter = 0;
else
    iter = iter + 1;
end

w0 = param.wref(iter*10+1:(iter+N)*10+8);


% objective function (w contains N+1 x vectors and N u vectors)
%objFunc = @(w) objFuncN(w, N);

% linear inequality constraint
stateTol = param.tolerances.state(1:8);
inputTol = param.tolerances.input(1:2);
ropeLen =  param.craneParams.r;
rectConstr = param.constraints.rect;
ellConstr = param.constraints.ellipses;
n_at_equilibrium = max(0, iter - param.Tf / param.Ts + 1 + N);
[A, b] = inequalityConstraints(N, r, stateTol, inputTol, ropeLen, rectConstr, ellConstr, w0, n_at_equilibrium);

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

% penBlock = [ones(10,1); zeros(10*(param.TsFactor-1),1)];
% n_blocks = N/param.TsFactor;
% penalties = [kron(ones(n_blocks,1), penBlock); ones(8,1)];
penalties = ones(10*N+8, 1);
H = diag(penalties);

refTraj = param.wref(iter*10+1:(iter+N)*10+8);
f = - H * refTraj;
w = quadprog(H,f,A,b,Aeq,beq);

% extract u from w
u = w(9:10);

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
function [A, b] = inequalityConstraints(N, r, tolState, tolInput, ropeLen, rectConstraints, ellipses, w, n_final)
    if n_final < N + 1
        A = zeros(0, 8+10*N);
        b = zeros(0, 1);
    else
        A = [ zeros(4,8), [eye(2); -eye(2)], zeros(4,10*(N-1)+8) ];
        b = [ tolInput; tolInput ];
    end
    
    for i=1:N-1
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen, w(10*i+5), w(10*i+7));
        
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ropeLen, ellipses, w(10*i+1), w(10*i+3), w(10*i+5), w(10*i+7));
        
        
        A_tmp = [A1;A2;A3];
        A_tmp2 = [zeros(size(A_tmp,1), i*10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10-10)];
        A = [A; A_tmp2];
        b = [b; b1;b2;b3];
        
        if N - i < n_final
            % final state/input constraint
            [A4, b4] = finalRows(r, tolState, tolInput, i, N);
            A = [A; A4];
            b = [b; b4];
        end
    end
    
    if n_final > 0
        % final state/input constraint
        [A_tmp, b_tmp] = finalRows(r, tolState, tolInput, N, N);
        A = [A; A_tmp];
        b = [b; b_tmp];
    end
end

function out = linePars(a,b) 
    if a(1) == b(1)
    out = [-1, 0, -a(1)];
    else
    tmp = polyfit([a(1) b(1)],[a(2) b(2)],1);
    out = [-tmp(1) 1 tmp(2)];
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
    ARows(1,9) =  1; % u < 1
    ARows(2,9) = -1; % u > 1
    ARows(3,10) =  1; % u < 1
    ARows(4,10) = -1; % u > 1
    bRows = [1;1;1;1];
end

function [ARows, bRows] = rectLimsRows(rectConstraints, ropeLen, thetag, phig)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    D = [ DRect(1,1) 0 DRect(1,2) 0 0             0 0             0 0 0; 
          DRect(2,1) 0 DRect(2,2) 0 0             0 0             0 0 0; 
          DRect(1,1) 0 DRect(1,2) 0 bx*DRect(1,1) 0 by*DRect(1,2) 0 0 0;
          DRect(2,1) 0 DRect(2,2) 0 by*DRect(2,1) 0 by*DRect(2,2) 0 0 0 ];
    ARows = [D; -D];
    offset = [ - ax * DRect(1,1) - ay * DRect(1,2);
               - ax * DRect(2,1) - ay * DRect(2,2) ];
    bRows = [ chRect; 
              chRect + offset ; 
              -clRect;
              -clRect - offset ];
end

function [ARows, bRows] = ellipseLimsRows(ropeLen, ellipses, xg, yg, thetag, phig)
    ARows = zeros(0,10);
    bRows = zeros(0,1);
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            [ARow1, bRow1] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b);
            [ARow2, bRow2] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, ell.xc, ell.yc, ell.a, ell.b);
            ARows = [ARows; ARow1; ARow2];
            bRows = [bRows; bRow1; bRow2];
        end
    end
end

function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b)
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ARow = [ -beta, 0, -gamma, 0, 0, 0, 0, 0, 0, 0 ];
    bRow = alpha - beta * xg - gamma * yg;
end

function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b)
    xg_p = xg + ropeLen * sin(thetag); 
    yg_p = yg + ropeLen * sin(phig); 
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a, b);
    beta =  2 * (xg_p - xc) / (a^2);
    gamma = 2 * (yg_p - yc) / (b^2);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARow = [ -beta, 0, -gamma, 0, -beta*bx, 0, -gamma*by, 0, 0, 0 ];
    bRow = alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay;
end

function [ARows, bRows] = finalPositionRows(r, tolerances, i, N)
    selPos = [1 0 0 0 0 0 0 0; 
              0 0 1 0 0 0 0 0];
    ARows = [ zeros(4, 10*i), [selPos; -selPos], zeros(4, 10*N-10*i) ];
    bRows = [ r(1,:) + tolerances(1,:)/2; 
              r(3,:) + tolerances(3,:)/2; 
              - r(1,:) + tolerances(1,:)/2; 
              - r(3,:) + tolerances(3,:)/2 ];
end

function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N)
    if i == N
        ARows = [ zeros(16, 10*i), [eye(8); -eye(8)] ];
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        ARows = [ zeros(20, 10*i), [eye(10); -eye(10)], zeros(20, 10*N-10*i-2) ];
        bRows = [ r + tolerancesState/2;
                  tolerancesInput;
                  - r + tolerancesState/2;
                  tolerancesInput ];
    end
end




%% linear equality constraints
function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, Ts)
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
       
    for i=1:N
       x = xus(i*10-9:i*10-2);
       u = xus(i*10-1:i*10);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, i*10-9:i*10-2) = A; 
       Aeq(8+i*8-7:8+i*8, i*10-1:i*10) = B;
       Aeq(8+i*8-7:8+i*8, i*10+1:i*10+8) = - eye(8);
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
    ceq = w2x(w) - doStep(w, dt, Ts); 
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

function w = runInitialOptimisation(finalTrgt, x_hat, param, factorTs)

% horizon length (prediction AND control)
Ts = param.Ts * factorTs; % initial guess to uses longer Ts to reduce set-up time
N = param.Tf / Ts;

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
nonlcon = @(w) nonLinearConstraints(param.constraints.rect, param.craneParams.r, true, param.constraints.ellipses, param.modelDerivative, Ts, w);

% options
options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);

% optimisation
A = sparse(A);
Aeq = sparse(Aeq);
w = fmincon(objFunc,w0,A,b,Aeq,beq,[],[],nonlcon,options);

% interpolate 
w = w(1:end-10);
w = interpolate(w, factorTs);

end 

function [Aeq, beq] = linearEqConstrInitialGuess(x0, w0, genA, genB, der, N, Ts, r)
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8) = x0;

    % final position constraint
    [A_tmp, b_tmp] = finalPositionRowsInitGuess(r, N);
    Aeq = [Aeq; A_tmp];
    beq = [beq; b_tmp];
end

function [A, b] = inequalityConstraintsInitialGuess(N, ropeLen, rectConstraints, r, stateTol, inputTol)
    A = zeros(0, 8+10*N);
    b = zeros(0, 1);
    
    for i=1:N-1
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRowsInitialGuess(rectConstraints);
        
        % combine matrices
        A_tmp = [A1;A2];
        A_tmp2 = [zeros(size(A_tmp,1), i*10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10-10)];
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
    D = [ DRect(1,1) 0 DRect(1,2) 0 0             0 0             0 0 0; 
          DRect(2,1) 0 DRect(2,2) 0 0             0 0             0 0 0 ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
end
