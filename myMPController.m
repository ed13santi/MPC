function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(6,1);
%

%convert state to MPC's format
x_MPC = x_hat(1:8);


persistent t
if isempty(t)
    t = 0;
else
    t = t + param.Ts;
end

%horizon length
N = max(10, floor((param.Tf - t) / param.Ts));

% Declare penalty matrices: 
P = diag([0,0,0,0,0,0,0,0]);
Q = diag([0,0,0,0,0,0,0,0]);
R = [ 0,0,0,0,0,0;    % u_x
      0,0,0,0,0,0;    % u_y
      0,0,0,0,1,0;    % slack var for |u_x|
      0,0,0,0,0,1;    % slack var for |u_y|
      0,0,1,0,0,0;    % slack var for |x dot|
      0,0,0,1,0,0 ];  % slack var for |y dot|

M = blkdiag(Q,R);
H = blkdiag( kron(eye(N), M), P );

f = zeros( 1, length(u)*N + 8*(N+1) );

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

g = [x_MPC; zeros(8*N,1)];


% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);

% rectangle constraints
D = [ [DRect(1,1) 0 DRect(1,2) 0 0 0 0 0];  %constraints on pair of sides of rectangle
      [DRect(2,1) 0 DRect(2,2) 0 0 0 0 0];  %constraints on pair of sides of rectangle
      [0 0 0 0 1 0 0 0];                    %limit on Theta
      [0 0 0 0 0 0 1 0] ];                  %limit on Phi
  

ang_lim = 4*pi/180;
cl = [clRect; -ang_lim; -ang_lim];
ch = [chRect; ang_lim; ang_lim];

E =      [1  0  0  0  0  0;    % -1 < u_x < 1
          0  1  0  0  0  0;    % -1 < u_y < 1
          0  0 -1  0  0  0;    % 0 < slack|u_x|
          0  0  0 -1  0  0;    % 0 < slack|u_y|
          0  0  0  0 -1  0;    % 0 < slack|xdot|
          0  0  0  0  0 -1;    % 0 < slack|ydot|
          1  0 -1  0  0  0;    % u_x < slack|u_x|
         -1  0 -1  0  0  0;    % -u_x < slack|u_x|
          0  1  0 -1  0  0;    % u_y < slack|u_y|
          0 -1  0 -1  0  0 ];  % -u_y < slack|u_y|

ul = [-1; -1];
uh = [1; 1; 0; 0; 0; 0; 0; 0; 0; 0];
  
% Compute stage constraint matrices and vector
[Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh);


% add relationship between x and u
Dt = [Dt; 
     [0  1 0  0 0 0 0 0;    % xdot < slack|xdot|
      0 -1 0  0 0 0 0 0;    % -xdot < slack|xdot|
      0  0 0  1 0 0 0 0;    % ydot < slack|ydot|
      0  0 0 -1 0 0 0 0 ]]; % -ydot < slack|ydot|
  
Et = [Et; 
     [0 0 0 0 -1  0;
      0 0 0 0 -1  0;
      0 0 0 0  0 -1;
      0 0 0 0  0 -1 ]];

bt = [bt; zeros(4,1)];

% Compute trajectory constraints matrices and vector
[D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N);

tolX = param.tolerances.state(1)/2;
tolY = param.tolerances.state(2)/2;

[DTrgt,chTrgt,clTrgt] = rectConstraints([ r(1)-tolX, r(2)-tolY;
                                          r(1)+tolX, r(2)-tolY;
                                          r(1)+tolX, r(2)+tolY;
                                          r(1)-tolX, r(2)+tolY ]);
                                      
        
% rectangle constraints
Dl = [ [DTrgt(1,1) 0 DTrgt(1,2) 0  0  0  0  0];   % constraints on pair of sides of rectangle
       [DTrgt(2,1) 0 DTrgt(2,2) 0  0  0  0  0];   % constraints on pair of sides of rectangle
       [0          1 0          0  0  0  0  0];   % xdot < tol
       [0          0 0          1  0  0  0  0];   % ydot < 
       [0          0 0          0  1  0  0  0];   % ang1 <
       [0          0 0          0  0  1  0  0];   % ang1dot <
       [0          0 0          0  0  0  1  0];   % ang2 <
       [0          0 0          0  0  0  0  1];   % ang2dot <
       [0         -1 0          0  0  0  0  0];   % same but other sign
       [0          0 0         -1  0  0  0  0];   %
       [0          0 0          0 -1  0  0  0];   %
       [0          0 0          0  0 -1  0  0];   %
       [0          0 0          0  0  0 -1  0];   %
       [0          0 0          0  0  0  0 -1] ]; %
   
tol = param.tolerances.state;

D = blkdiag(D, Dl);
E = [E; zeros(size(Dl,1),size(E,2))];
d = [d; clRect; tol(2); tol(4:8); tol(2); tol(4:8)];


% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
% [Lchol,p] = chol(H,'lower');
% Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
%u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,length(u));
u = genMPControllerSparse(H,f,G,g,D,d,length(u),N);

end % End of myMPController







%% OTHER FUNCTIONS

