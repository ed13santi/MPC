function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(4,1);
%

%convert state to MPC's format
x_MPC = x_hat(1:8);

%horizon length
N = 25;

%Declare penalty matrices: 
eps = 1e-10;
lambda = 1e5; %coefficient of work soft constraint
P = 100000 * diag([1,1,1,1,eps,eps,eps,eps]);
Q = 100000 * diag([1,1,1,1,eps,eps,eps,eps]);
R = diag([0.001;0.001;lambda;lambda]);

A = param.A;
B = [param.B, zeros(8,length(u)-2)];
C = param.C;

% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);
D = [];

% rectangle constraints
D = [D; [DRect(1,1) 0 DRect(1,2) 0 0 0 0 0]]; 
D = [D; [0 DRect(2,1) 0 DRect(2,2) 0 0 0 0]]; 

D = [D; [0 0 0 0 1 0 0 0]]; %limit on Theta
D = [D; [0 0 0 0 0 0 1 0]]; %limit on Phi
E = [ 1       , 0       , 0 ,  0;   % -1 < u_x < 1
      0       , 1       , 0 ,  0;   % -1 < u_y < 1
      x_hat(2), 0       , -1,  0;   % x_dot * u_x < gamma_x
      0       , 0       , -1,  0;   % 0 < gamma_x
      0       , x_hat(4), 0 , -1;   % y_dot * u_y < gamma_y
      0       , 0       , 0 , -1 ]; % 0 < gamma_y

% Compute stage constraint matrices and vector
ang_lim = 4*pi/180;
cl = [clRect; -ang_lim; -ang_lim];
ch = [chRect; ang_lim; ang_lim];
ul = [-1; -1];
uh = [1; 1; 0; 0; 0; 0];
[Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh);

% Compute trajectory constraints matrices and vector
[DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N);


%add constraint on sum of gammas
DD = [ DD; zeros(1, size(DD,2)) ];
tmp_row = zeros(1, size(EE,2));
for i=3:length(uh):size(EE,2)
    tmp_row(i) = 1;
    tmp_row(i + 1) = 1;
end
EE = [ EE; - tmp_row ]; % sum of gammas > work limit


bb = [bb; - param.Wmax * N / param.Tf];

% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N);
[F,J,L]=genConstraintMatrices(DD,EE,Gamma,Phi,N,5);

% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);

% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
% [Lchol,p] = chol(H,'lower');
% Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
%u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,length(u));
u = genMPController(H,G,F,bb,J,L,x_MPC,r,length(u),N);


u = u(1:2);

end % End of myMPController







%% OTHER FUNCTIONS

