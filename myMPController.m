function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%

%convert state to MPC's format
x_MPC = x_hat(1:8);

%horizon length
N = 15;

%Declare penalty matrices:      
%if abs(x_MPC - r) < param.tolerances.state(1:8)
%    P = diag([1,1,1,1,1,1,1,1])
%    Q = diag([1,1,1,1,1,1,1,1])
%else
P = 100000 * diag([1,1,1,1,0,0,0,0]);
Q = 100000 * diag([1,1,1,1,0,0,0,0]);
%end
       
R = 0.00000001 * diag([1;1]);

A = param.A;
B = param.B;
C = param.C;

% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);
D = zeros(2, size(param.A,1));
D(1,1) = DRect(1,1);
D(2,1) = DRect(2,1);
D(1,3) = DRect(1,2);
D(2,3) = DRect(2,2);
D(3,5) = 1; %limit on Theta
D(4,7) = 1; %limit on Phi

% Compute stage constraint matrices and vector
ang_lim = 0.01*pi;
cl = [clRect; -ang_lim; -ang_lim];
ch = [chRect; ang_lim; ang_lim];
ul = [-1; -1];
uh = [1; 1];
[Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh);

% Compute trajectory constraints matrices and vector
[DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N);

% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N);
[F,J,L]=genConstraintMatrices(DD,EE,Gamma,Phi,N);

% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);       
% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
[Lchol,p] = chol(H,'lower');
Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,2);

u = u(1:2);

end % End of myMPController







%% OTHER FUNCTIONS

