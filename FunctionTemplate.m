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

% Horizon length
param.N = 15;

% How much before Ts to contraint final position
param.advance = 0.95;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

% Create generic state space that can be used at every time step using the
% current values of x and u
[param.genericA, param.genericB, param.modelDerivative] = obtainJacs(param.craneParams);
%load('Crane_NominalParameters.mat');
%[param.A,param.B,param.C,~] = genModel(m,M,MR,r,9.81,Tx,Ty,Vx,Vy,param.Ts);

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
delta_x = param.xTar - x_initial(1);
delta_y = param.yTar - x_initial(3);
r = zeros(8,1);
if t_target >= t_reach
    % Make the crane go to (xTar, yTar)
    r(1,1) = param.xTar;
    r(3,1) = param.yTar;
else
    ratio = t_target / t_reach;
    r(1,1) = x_initial(1) + ratio * delta_x;
    r(3,1) = x_initial(3) + ratio * delta_y;
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




%% Modify the following function for your controller
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
    w0(1:end-13) = w0(14:end);
end

% linear inequality constraint
[A, b] = inequalityConstraints(N, r, param.tolerances.state(1:8));

% linear equality constraints (currently only equality constraint on x0)
[Aeq, beq] = getStateSpace(x_hat, w0, param.genericA, param.genericB, param.modelDerivative, N, param.craneParams.r, param.Ts);
%[Aeq, beq] = linearConstraintsSimple(param.A, param.B, x_hat, N, param.craneParams.r, r);

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
penalties = zeros(10+13*N,1);
for i=1:N
    penalties(13*i-2:13*i-1) = ones(2,1); 
end
w = quadprog(diag(penalties),zeros(1,size(A,2)),A,b,Aeq,beq);

% extract u from w
u = w(11:12);

end % End of myMPController







%% objective function

function out = objFuncN(w, N)
    penalties = zeros(10+13*N,1);
    for i=1:N
       penalties(13*i-2:13*i-1) = ones(2,1); 
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
function [A, b] = inequalityConstraints(N, r, tolerances)
    A = zeros(4*N+8*2, 10+13*N);
    b = zeros(4*N+8*2, 1);
    for i=1:N
        A(4*i-3,13*i-2) =  1; % u < 1
        A(4*i-2,13*i-2) = -1; % u > 1
        A(4*i-1,13*i-1) =  1; % u < 1
        A(4*i  ,13*i-1) = -1; % u > 1
        b(4*i-3:4*i) =  [1;1;1;1];
    end
    A(4*N+1:+4*N+8 , 13*N+1:13*N+8) = eye(8);
    A(4*N+9:+4*N+16, 13*N+1:13*N+8) = -eye(8);
    b(4*N+1:+4*N+8)  = r + tolerances/2;
    b(4*N+9:+4*N+16) = tolerances/2 - r;
end

function [A, b] = inequalityConstraintsEASY(N, r, tolerances)
    A = zeros(8*2, 10+13*N);
    b = zeros(8*2, 1);
    
    A(end-15:end-8, end-9:end-2) = eye(8);
    A(end-7:end, end-9:end-2) = -eye(8);
    b(end-15:end-8) = r + tolerances;
    b(end-7:end) = tolerances - r;
end



%% linear equality constraints
function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, radius, Ts)
    xus = [x0; radius; 0; w0(11:end-10)];
    x = zeros(N*10, 1);
    u = zeros(N*3, 1);
    Aeq = zeros(10+(10+2)*N, length(w0));
    beq = zeros(10+(10+2)*N, 1);
    
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; radius; 0];
    for i=1:N
       x = xus(13*i-12:13*i-3);
       u = xus(13*i-2:13*i);
       funcInp = [x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), u(1), u(2), u(3)];
       A = genA(funcInp);
       B = genB(funcInp);
       Aeq(10+i*12-11:10+i*12-2, i*13-12:i*13-3) = eye(length(x)) + Ts*A/2; 
       Aeq(10+i*12-11:10+i*12-2, i*13-2:i*13) = Ts*B;
       Aeq(10+i*12-11:10+i*12-2, i*13+1:i*13+10) = Ts*A/2 - eye(length(x));
       Aeq(10+i*12-1 :10+i*12, i*13+9:i*13+10) = eye(2); % r and r_dot constraints
       beq(10+12*i-11:10+12*i-2) = - Ts * (der(funcInp) - A*x - B*u); % b (signs reversed cuz on other side of eqn) MUST USE the continuous A B
       beq(10+12*i-1 :10+12*i) = [radius; 0]; % r and r_dot constraints
    end
end

function [Aeq, beq] = linearConstraintsSimple(A, B, x0, N, r, trgt)
    Aeq = zeros(10+(10+2)*N, 13*N+10);
    beq = zeros(10+(10+2)*N, 1);
        
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; r; 0];
    for i=1:N
       Aeq(10+i*12-11:10+i*12-2, i*13-12:i*13-3) = blkdiag(A,1,1);
       Aeq(10+i*12-11:10+i*12-2, i*13-2 :i*13) = [[B, zeros(size(B,1),1)]; zeros(2,size(B,2)+1)];
       Aeq(10+i*12-11:10+i*12-2, i*13+1 :i*13+10) = - eye(size(A,1)+2);
       Aeq(10+i*12-1 :10+i*12  , i*13+9 :i*13+10) = eye(2); % r and r_dot constraints
       beq(10+i*12-11:10+i*12-2) = zeros(10,1); % b (signs reversed cuz on other side of eqn) MUST USE the continuous A B
       beq(10+i*12-1 :10+i*12) = [r; 0]; % r and r_dot constraints
    end
end


%% model linearisation
function [genericA, genericB, der] = obtainJacs(cP)

    syms dx x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 fx fy fl 
    g  = 9.81;
    m  = cP.m;
    Tl = cP.Tl;
    Ty = cP.Ty;
    Tx = cP.Tx;
    M  = cP.M;
    MR = cP.MR;
    
    dx = sym(zeros(10, 1));

    dx(1, 1)  = x2;
    dx(2, 1)  = (cos(x5) .* cos(x7) .^ 2 .* sin(x5) .* 9.81 .* m + sin(x5) .* (Tl .* x10 - fl) .* cos(x7) - Tx .* x2 + fx) ./ (M + MR);
    dx(3, 1)  = x4;
    dx(4, 1)  = ((m .* g .* cos(x5) .* cos(x7) + Tl .* x10 - fl) .* sin(x7) - Ty .* x4 + fy) ./ M;
    dx(5, 1)  = x6;  
    dx(6, 1)  = (-cos(x5) .^ 2 .* cos(x7) .^ 2 .* sin(x5) .* g .* m + (-sin(x5) .* (Tl .* x10 - fl) .* cos(x5) - 0.2e1 .* x6 .* x10 .* (M + MR)) .* cos(x7) + (Tx .* x2 - fx) .* cos(x5) + 0.2e1 .* (M + MR) .* (sin(x7) .* x6 .* x8 .* x9 - sin(x5) .* g ./ 0.2e1)) ./ x9 ./ cos(x7) ./ (M + MR);
    dx(7, 1)  = x8;
    dx(8, 1)  = (-cos(x5) .* g .* sin(x7) .* m .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 2 + ((-M .* (Tl .* x10 - fl) .* cos(x5) .^ 2 - M .^ 2 .* x6 .^ 2 .* x9 - x6 .^ 2 .* x9 .* MR .* M - MR .* (Tl .* x10 - fl)) .* sin(x7) - (-Ty .* x4 + fy) .* (M + MR)) .* cos(x7) - 0.2e1 .* ((g .* (M + MR) .* cos(x5) ./ 0.2e1 + sin(x5) .* (Tx .* x2 - fx) ./ 0.2e1) .* sin(x7) + x10 .* x8 .* (M + MR)) .* M) ./ x9 ./ M ./ (M + MR);
    dx(9, 1)  = x10;
    dx(10, 1) = (cos(x5) .* g .* m .^ 2 .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 3 + m .* (M .* (Tl .* x10 - fl) .* cos(x5) .^ 2 + M .^ 2 .* x6 .^ 2 .* x9 + x6 .^ 2 .* x9 .* MR .* M + MR .* (Tl .* x10 - fl)) .* cos(x7) .^ 2 + (-g .* m .* (M + MR) .* cos(x5) + M .* sin(x5) .* (Tx .* x2 - fx)) .* m .* cos(x7) + (M + MR) .* (-m .* (-Ty .* x4 + fy) .* sin(x7) + (M .* x8 .^ 2 .* x9 - Tl .* x10 + fl) .* m - M .* (Tl .* x10 - fl))) ./ m ./ M ./ (M + MR);
        
    jacA = jacobian(dx, [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10]);
    jacB = jacobian(dx, [fx fy fl]);

    genericA = @(w) double(subs(jacA, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, fx, fy, fl], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
    genericB = @(w) double(subs(jacB, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, fx, fy, fl], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
    der      = @(w) double(subs(dx  , [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, fx, fy, fl], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
end

function [A,B,C,D] = genModel(m,M,MR,r,g,Tx,Ty,Vx,Vy,Ts)
    % replace A,B,C,D with the correct values
    A=[ 0, 1,             0, 0,        0,                          0, 0,              0;
        0, -Tx/(M+MR),    0, 0,        g*m/(M+MR),                 0, 0,              0;
        0, 0,             0, 1,        0,                          0, 0,              0;
        0, 0,             0, -Ty/M,    0,                          0, g*m/M,          0;
        0, 0,             0, 0,        0,                          1, 0,              0;
        0, Tx/(r*(M+MR)), 0, 0,        -(g*(m+M+MR))/(r*(M+MR)),   0, 0,              0;
        0, 0,             0, 0,        0,                          0, 0,              1;
        0, 0,             0, Ty/(M*r), 0,                          0, -g*(m+M)/(M*r), 0 ];
    
    B=[ 0,              0;
        Vx/(M+MR),      0;
        0,              0;
        0,              Vy/M;
        0,              0;
        -Vx/(r*(M+MR)), 0; 
        0,              0; 
        0,              -Vy/(M*r) ];
    
    C=eye(8);
    
    D=zeros(8,2);
    
    % if Ts>0 then sample the model with a zero-order hold (piecewise constant) input, otherwise return a continuous-time model
    if Ts>0
        sys  = ss(A,B,C,D);
        sysd = c2d(sys, Ts);
        A = sysd.A;
        B = sysd.B;
        C = sysd.C;
        D = sysd.D;
    end
end
