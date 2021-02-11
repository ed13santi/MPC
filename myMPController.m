function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%%

%horizon length
N = 15;

%u = param.K*(r - x_hat);

% Declare penalty matrices:
Q = 1*diag(1:8);
P = 100000*diag(3:10);
R = diag([2;3]);

A = param.A;
B = param.B;
C = param.C;

% Constraints %SHOULD THE TARGET BE SPECIFIED AS A CONSTRAINT?
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);
D = zeros(2, size(param.A,1));
D(1,1) = DRect(1,1);
D(2,1) = DRect(2,1);
D(1,3) = DRect(1,2);
D(2,3) = DRect(2,2);

% Compute stage constraint matrices and vector
cl = clRect;
ch = chRect;
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
u = genMPController(Linv,G,F,bb,J,L,x_hat,r,2);

end % End of myMPController







%% OTHER FUNCTIONS

