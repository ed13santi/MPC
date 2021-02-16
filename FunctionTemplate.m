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

% Load model parameters and calculate matrices
load('Crane_NominalParameters.mat');
[param.A,param.B,param.C,~] = genCraneODE(m,M,MR,r,9.81,Tx,Ty,Vx,Vy,param.Ts);

end % End of mySetup






function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
r = zeros(8,1);

% Make the crane go to (xTar, yTar)
r(1,1) = param.xTar;
r(3,1) = param.yTar;

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
u = zeros(4,1);
%

%convert state to MPC's format
x_MPC = x_hat(1:8);

% horizon length
N = 25;

% Declare penalty matrices: 
eps = 1e-10;
lambda = 1e15; %coefficient of work soft constraint
P = 100000 * diag([1,1,1,1,eps,eps,eps,eps]);
Q = 100000 * diag([1,1,1,1,eps,eps,eps,eps]);
R = diag([0.001;0.001;lambda;lambda]);

M = blkdiag(Q,R);
H = blkdiag( kron(eye(N), M), P );

offsetOneTimeStep = [ - r(1); 0; - r(1); 0; 0; 0; 0; 0; zeros(length(u),1) ];
offsetMatrix = [ kron(ones(N,1), offsetOneTimeStep);
                 [ - r(1); 0; - r(1); 0; 0; 0; 0; 0] ];
f = H' * offsetMatrix;



% Equality constraints - model physics
A = param.A;
B = [param.B, zeros(8,length(u)-2)];
C = param.C;

step = size(A,1);
h_step = size(A,2) + size(B,2);

G_top = [eye(step), zeros(step, N * h_step)];
G_bot = zeros(N * step, N * h_step + step);
G_block = [-A, -B, eye(8)];
for i=0:N-1
    h_start = h_step*i+1;
    h_end = h_step*(i+1)+step;
    v_start = step*i+1;
    v_end = step*(i+1);
    G_bot(v_start:v_end,h_start:h_end) = G_block;
end
G = [G_top; G_bot];

g = [x_hat; zeros(N * step, 1)];



% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);
D = [];

% rectangle constraints
D = [D; [DRect(1,1) 0 DRect(1,2) 0 0 0 0 0]]; 
D = [D; [0 DRect(2,1) 0 DRect(2,2) 0 0 0 0]]; 

D = [D; [0 0 0 0 1 0 0 0]]; %limit on Theta
D = [D; [0 0 0 0 0 0 1 0]]; %limit on Phi

E = [ 1        , 0       , 0 ,  0;   % -1 < u_x < 1
      0        , 1       , 0 ,  0;   % -1 < u_y < 1
      x_hat(2) , 0       , -1,  0;   % x_dot * u_x < gamma_x
      0        , 0       , -1,  0;   % 0 < gamma_x
      0        , x_hat(4), 0 , -1;   % y_dot * u_y < gamma_y
      0        , 0       , 0 , -1 ]; % 0 < gamma_y

  
% Compute stage constraint matrices and vector
ang_lim = 4*pi/180;
cl = [clRect; -ang_lim; -ang_lim];
ch = [chRect; ang_lim; ang_lim];
ul = [-1; -1];
uh = [1; 1; 0; 0; 0; 0];
[Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh);



% Compute trajectory constraints matrices and vector
[D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N);


%add constraint on sum of gammas
tmp_row = zeros(1, size(D,2));
for i=8+3:8+length(uh):size(D,2)
    tmp_row(i) = 1;
    tmp_row(i + 1) = 1;
end
D = [ D; - tmp_row ]; % sum of gammas > work limit

d = [d; - param.Wmax * N / param.Tf];

% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
% [Lchol,p] = chol(H,'lower');
% Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
%u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,length(u));
u = genMPControllerSparse(H,f,G,g,D,d,length(u),N);

end % End of myMPController







%% OTHER FUNCTIONS

function [A,B,C,D] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vx,Vy,Ts)
% Inputs:
% m = Pendulum mass (kg)
% M = Cart mass (kg)
% MR = Rail mass (kg)
% r = String length (m)
% g = gravitational accelaration (m*s^-2)
% Tx = Damping coefficient in X direction (N*s*m^-1)
% Ty = Damping coefficient in Y direction (N*s*m^-1)
% Vm = Input multiplier (scalar)
% Ts = Sample time of the discrete-time system (s)
% Outputs:
% A,B,C,D = State Space matrices of a discrete-time or continuous-time state space model

% The motors in use on the gantry crane are identical and therefore Vx=Vy.

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
    sys = ss(A,B,C,D);
    sysd = c2d(sys, Ts);
    A = sysd.A;
    B = sysd.B;
    C = sysd.C;
    D = sysd.D;
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

function [mat, ch, cl] = rectConstraints(rect)
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




function u = genMPControllerSparse(H,f,G,g,D,d,m,N)
%opt.MaxIterations = 200;
%opt.IntegrityChecks = false;%% for code generation
%opt.ConstraintTolerance = 1e-3;
%opt.DataType = 'double';
%opt.UseHessianAsInput = false;
%% your code starts here
%persistent iA
%if isempty(iA)
%    iA = 
%end
%[U,~,iA,~] = mpcActiveSetSolver(H, linTerm, F, rightIneqConstr, [], zeros(0,1), false(size(bb)), opt);

persistent w0
if isempty(w0)
    u0 = zeros(m*N+8*(N+1),1);
end
%options =  optimset('Display', 'on','UseHessianAsInput','False');
W = quadprog(H, f, D, d, G, g, [], [], u0);
%% your remaining code here
u = W(9:10);
W0 = W;
end

function [Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh) 
%modified version, the input constraints are now ul <= Eu <= uh
    Dt = [D*A; -D*A; zeros(length(uh) + length(ul),size(A,2))];
    lower = zeros(length(ul), size(E,2));
    for i=1:length(ul)
        lower(i,i) = -1;
    end
    Et = [D*B; -D*B; E; lower];
    bt = [ch; -cl; uh; -ul];
end

function [D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N)
% your code goes here
block = [Dt, Et];
D = blkdiag(kron(eye(N), block), Dt);
d = kron(ones(N+1,1), bt);
end


