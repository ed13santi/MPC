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
param.Tf = shape.Tf;    

% This is how to set the sampling interval
param.Ts = 0.1;

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% This is a sample way to send reference points
initialState = [shape.start(1); 0; shape.start(2); 0; 0; 0; 0; 0];
targetState  = [param.xTar;     0; param.yTar;     0; 0; 0; 0; 0];

% Horizon length
param.N = 15;

% How much before Ts to contraint final position
param.advance = 0.95;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

% Create generic state space that can be used at every time step using the
% current values of x and u
[param.genericA, param.genericB, param.modelDerivative] = obtainJacs(param.craneParams);

%Run initial optimisation over whole length of episode using fully
%non-linear model
param.w_guess = runInitialOptimisation(targetState, initialState, param);

figure;
plotx = [];
ploty = [];
for i=1:(length(param.w_guess)-8)/10
   plotx = [plotx param.w_guess(i*10+1)]; 
   ploty = [ploty param.w_guess(i*10+3)]; 
end
scatter(plotx, ploty);

end % End of mySetup

function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

persistent t
if isempty(t)
    t = 0;
else
    t = t + param.Ts;
end

% initial x
persistent x_initial
if isempty(x_initial)
    x_initial = x_hat;
end

t_target = t + param.N * param.Ts;
t_reach = param.advance * param.Tf;
r = zeros(8,1);
if t_target >= t_reach
    % Make the crane go to (xTar, yTar)
    r(1,1) = param.xTar;
    r(3,1) = param.yTar;
else
    trgt_index = t_target / param.Ts + 1;
    r = param.w_guess(trgt_index*10-9:trgt_index*10-2);
end

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

function out = objFuncN(w, N)
    penalties = zeros(8+10*N,1);
    for i=1:N
       penalties(10*i-1:10*i) = ones(2,1); 
    end
    out = w' * diag(penalties) * w;
%     vels = zeros(2*(N+1), 1);
%     us = zeros(2*N, 1);
%     for i = 1:N
%         vels(2*i-1:2*i, 1) = w(13*i-11:2:13*i-9);
%         us(2*i-1:2*i, 1) = w(13*i-2:13*i-1);
%     end
%     vels(2*(N+1)-1:2*(N+1), 1) = w(13*(N+1)-11:2:13*(N+1)-9);
%     
%     vels = vels(3:end,1) + vels(1:end-2,1);
%     prods = us .* vels;
%     maxed = max(prods,[],2);
%     out = sum(maxed);
end

function out = objFuncInitialGuess(w, N)
    penalties = zeros(8+10*N,1);
    for i=1:N
       penalties(10*i-1:10*i) = ones(2,1); 
    end
    out = w' * diag(penalties) * w;
%     vels = zeros(2*(N+1), 1);
%     us = zeros(2*N, 1);
%     for i = 1:N
%         vels(2*i-1:2*i, 1) = w(13*i-11:2:13*i-9);
%         us(2*i-1:2*i, 1) = w(13*i-2:13*i-1);
%     end
%     vels(2*(N+1)-1:2*(N+1), 1) = w(13*(N+1)-11:2:13*(N+1)-9);
%     
%     vels = vels(3:end,1) + vels(1:end-2,1);
%     prods = us .* vels;
%     maxed = max(prods,[],2);
%     out = sum(maxed);
end




%% linear inequality constraints
function [A, b] = inequalityConstraints(N, r, tolerances, ropeLen, rectConstraints, ellipses, w)
    A = zeros(0, 8+10*N);
    b = zeros(0, 1);
    
    for i=1:N
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen);
        
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ellipses, w(10*i-9), w(10*i-7));
        
        A_tmp = [A1;A2;A3];
        A_tmp2 = [zeros(size(A_tmp,1), i*10-10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10)];
        A = [A; A_tmp2];
        b = [b; b1;b2;b3];
    end
    
    % final position constraint
    [A_tmp, b_tmp] = finalPositionRows(r, tolerances, N);
    A = [A; A_tmp];
    b = [b; b_tmp];
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

function [ARows, bRows] = rectLimsRows(rectConstraints, ropeLen)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    D = [ DRect(1,1) 0 DRect(1,2) 0 0                  0 0                  0 0 0; 
          DRect(2,1) 0 DRect(2,2) 0 0                  0 0                  0 0 0;  
          DRect(1,1) 0 DRect(1,2) 0 ropeLen*DRect(1,1) 0 ropeLen*DRect(1,2) 0 0 0;
          DRect(2,1) 0 DRect(2,2) 0 ropeLen*DRect(2,1) 0 ropeLen*DRect(2,2) 0 0 0 ];
    ARows = [D; -D];
    bRows = [chRect; chRect; -clRect; -clRect];
end

function [ARows, bRows] = ellipseLimsRows(ellipses, xg, yg)
    ARows = zeros(0,10);
    bRows = zeros(0,1);
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            [ARow, bRow] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b);
            ARows = [ARows; ARow];
            bRows = [bRows; bRow];
        end
    end
end

function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b)
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ARow = [-beta 0 -gamma 0 0 0 0 0 0 0];
    bRow = alpha - beta * xg - gamma * yg;
end

function [ARows, bRows] = finalPositionRows(r, tolerances, N)
    ARows = [ zeros(16, 10*N), [eye(8); -eye(8)] ];
    bRows = [r + tolerances/2; tolerances/2 - r];
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



%% non-linear constraints

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
    out = ((x - xc)^2)/a^2 + ((y - yc)^2)/b^2 - 1;
end

function xVec = w2allX(w)
    N = (length(w)-8)/10;
    xVec = zeros(N,1);
    for i=1:N
        xVec(i) = w(10*i+1);
    end
end

function yVec = w2allY(w)
    N = (length(w)-8)/10;
    yVec = zeros(N,1);
    for i=1:N
        yVec(i) = w(10*i+3);
    end
end

function [c, ceq] = nonLinearConstraints(ellipses, dt, Ts, w)
    % inequality constraints for ellipses
    N = (length(w)-8)/10;
    c = zeros(N*size(ellipses,1)*size(ellipses,2),1);
    xVec = w2allX(w);
    yVec = w2allY(w);
    for i=1:N
        for j=1:size(ellipses,1)
            for z=1:size(ellipses,2)
                ell = ellipses{j,z};
                c(i) = - ellipseEval(xVec(i), yVec(i), ell.xc, ell.yc, ell.a, ell.b);
            end
        end
    end
    
    % equality constraints due to dynamics of system
    %ceq = w2x(w) - doStep(w, dt, Ts); % IF REACTIVATE THIS, NEED TO REMOVE
    %LINEAR MODEL CONSTRAINTS
    ceq = [];
end


%% run initial optimisation

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

function [Aeq, beq] = linearEqConstrInitialGuess(x0, w0, genA, genB, der, N, Ts)
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

function [A, b] = inequalityConstraintsInitialGuess(N, r, tolerances, ropeLen, rectConstraints, ellipses, w)
    n_ellipses = size(ellipses,1) * size(ellipses,2);
    n_each = 12 + n_ellipses;
    A = zeros(n_each*N+8*2, 8+10*N);
    b = zeros(n_each*N+8*2, 1);
    
    for i=1:N
        % input physical limits
        A(n_each*i-11,10*i-1) =  1; % u < 1
        A(n_each*i-10,10*i-1) = -1; % u > 1
        A(n_each*i-9 ,10*i)   =  1; % u < 1
        A(n_each*i-8 ,10*i)   = -1; % u > 1
        b(n_each*i-11:n_each*i-8) =  [1;1;1;1];
        
        % rectangle constraints
        [DRect,chRect,clRect] = rectCon(rectConstraints);
        D = [ DRect(1,1) 0 DRect(1,2) 0 0                  0 0                  0; 
              DRect(2,1) 0 DRect(2,2) 0 0                  0 0                  0;  
              DRect(1,1) 0 DRect(1,2) 0 ropeLen*DRect(1,1) 0 ropeLen*DRect(1,2) 0;
              DRect(2,1) 0 DRect(2,2) 0 ropeLen*DRect(2,1) 0 ropeLen*DRect(2,2) 0 ];
        A(n_each*i-7-n_ellipses:n_each*i-4-n_ellipses,10*i-9:10*i-2) = D;
        A(n_each*i-3-n_ellipses:n_each*i-n_ellipses,10*i-9:10*i-2) = -D;
        b(n_each*i-7-n_ellipses:n_each*i-4-n_ellipses) = [chRect; chRect];
        b(n_each*i-3-n_ellipses:n_each*i-n_ellipses) = [-clRect; -clRect];
        
    end
    
    % final position constraint
    A(12*N+1:12*N+8 , 10*N+1:10*N+8) = eye(8);
    A(12*N+9:12*N+16, 10*N+1:10*N+8) = -eye(8);
    b(12*N+1:12*N+8)  = r + tolerances/2;
    b(12*N+9:12*N+16) = tolerances/2 - r;
end
