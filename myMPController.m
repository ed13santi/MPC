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
P = 100000 * diag([1,1,1,1,eps,1,eps,1]);
Q = 100000 * diag([1,1,1,1,eps,eps,eps,eps]);
R = diag([0.001;0.001;eps;eps]);

M = blkdiag(Q,R);
H = blkdiag( kron(eye(N), M), P, lambda );

offsetOneTimeStep = [ - r(1); 0; - r(1); 0; 0; 0; 0; 0; zeros(length(u),1) ];
offsetMatrix = [ kron(ones(N,1), offsetOneTimeStep);
                 [ - r(1); 0; - r(1); 0; 0; 0; 0; 0];
                 0 ];
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
G = blkdiag(G, 0);

g = [x_hat; zeros(N * step, 1)];
g = [g; 0];



% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);

% rectangle constraints
D = [ [DRect(1,1) 0 DRect(1,2) 0 0 0 0 0]; 
      [0 DRect(2,1) 0 DRect(2,2) 0 0 0 0]; 
      [0 0 0 0 1 0 0 0]; %limit on Theta
      [0 0 0 0 0 0 1 0] ]; %limit on Phi

E = [ 1        , 0       , 0 ,  0;   % -1 < u_x < 1
      0        , 1       , 0 ,  0;   % -1 < u_y < 1
      %x_hat(2) , 0       , -1,  0;   % x_dot * u_x < gamma_x
      0        , 0       , -1,  0;   % 0 < gamma_x
      %0        , x_hat(4), 0 , -1;   % y_dot * u_y < gamma_y
      0        , 0       , 0 , -1 ]; % 0 < gamma_y

  
% Compute stage constraint matrices and vector
ang_lim = 4*pi/180;
cl = [clRect; -ang_lim; -ang_lim];
ch = [chRect; ang_lim; ang_lim];
ul = [-1; -1];
uh = [1; 1; 0; 0];% 0; 0];
[Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh);

% Compute non-linear constraints

% Compute trajectory constraints matrices and vector
[D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N);


%add constraint on sum of gammas
tmp_row = zeros(1, size(D,2)+1);
for i=8+3:8+length(uh):size(D,2)
    tmp_row(i) = 1;
    tmp_row(i + 1) = 1;
end
tmp_row(1,end) = - 1;
D = [ D, zeros(size(D,1),1) ];
D = [ D; - tmp_row ]; % sum of gammas < z work limit
D = [ D; [zeros(1, size(D,2)-1), -1] ]; % z > z work limit

d = [d; 0; - param.Wmax * N / param.Tf];

% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
% [Lchol,p] = chol(H,'lower');
% Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
%u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,length(u));
u = genMPControllerSparse(H,f,G,g,D,d,length(u),N);

end % End of myMPController







%% OTHER FUNCTIONS

