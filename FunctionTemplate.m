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
param.Ts = 0.05;

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% Horizon length
param.N = 15;

% How much before Ts to contraint final position
param.advance = 0.95;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

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

% horizon length
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

function out = objFuncN(w, N)
    out = 0;
    for i = 1:13:N*13
        out = out + workOfStep(w(i:i+22));
    end
end

function out = workOfStep(xuxVec)
    x = xuxVec(1:10);
    u = xuxVec(11:13);
    x_next = xuxVec(14:23);
    workX = max(0, u(1) * (x(2) + x_next(2)));                              % *Ts/2 is not needed
    workY = max(0, u(2) * (x(4) + x_next(4)));                              % *Ts/2 is not needed because minimisation is equivalent
    out = workX + workY;
end





%% linear equality constraints
function Aeq = leftEquality(N)
    selectLandLDot = [zeros(1,11), 1, 0; 
                      zeros(1,11), 0, 1];
    constraintLandLDots = kron(eye(N), selectLandLDot);
    Aeq = blkdiag(eye(10),constraintLandLDots);
end

function beq = rightEquality(N, x0, r)
    LandLDotConstraintSingle = [r;0];
    LandLDotConstraints = kron(ones(N+1,1), LandLDotConstraintSingle);
    beq = [x0; LandLDotConstraints];
end


%% non-linear constraints

function x = w2x(w)
    x = zeros((length(w)-10)/13*10, 1);
    for i=1:(length(w)-10)/13
        x(i*10-9:i*10) = w(i*13+1:i*13+10);
    end
end

function x = w2x_wave(w)
    x = zeros((length(w)-10)/13*10, 1);
    for i=1:(length(w)-10)/13
        x(i*10-9:i*10) = w(i*13-12:i*13-3);
    end
end

function u = w2u(w)
    u = zeros((length(w)-10)/13*3, 1);
    for i=1:(length(w)-10)/13
        u(i*3-2:i*3) = w(i*13-2:i*13);
    end
end

function x = doStep(x_wave, u, dt, craneParams) 
    x = x_wave;
    for i = 1:length(x_wave)/10
        x_before = x_wave(i*10-9:i*10);
        odeFun = @(t,y) crane_nl_model_student(u(i*3-2:i*3), y, craneParams);
        [~, y] = ode45(odeFun, [0 dt], x_before);
        x(i*10-9:i*10) = y(end,:);
    end
end

function [c, ceq] = nonLinearConstraints(dt, craneParams, w)
    c = [];
    ceq = w2x(w) - doStep(w2x_wave(w), w2u(w), dt, craneParams);
end




%% lower and upper bounds
function ub = ubConstraint(w_len, r, tolerances)
    ub = Inf * ones(w_len, 1);
    for i=0:13:w_len-13
        ub(i+11:i+12) = ones(2,1); % u < 1
    end
    ub(w_len-9:w_len-2) = r + tolerances; % final position constraints
end

function lb = lbConstraint(w_len, r, tolerances)
    lb = - Inf * ones(w_len, 1);
    for i=0:13:w_len-13
        lb(i+11:i+12) = - ones(2,1); % u > -1
    end
    lb(w_len-9:w_len-2) = r - tolerances; % final position constraints
end
