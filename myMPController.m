function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u_len = 2;
u = zeros(u_len,1);
%

% current time
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

%convert state to MPC's format
x_MPC = x_hat(1:8);

%horizon length
time_settle = param.advance*param.Tf;
time_left = time_settle - t;
samples_left = floor(time_left/param.Ts);
N = min(param.samples_max, max(param.samples_min, samples_left));

%Declare penalty matrices: 
%lambda = 1e3; %coefficient of work soft constraint

% if abs(x_MPC - r(1:8)) > param.tolerances.state(1:8);
%     P = diag([1,1,1,1,1,1,1,1]);
%     Q = diag([0,0,0,0,0,0,0,0]);
%     R = diag([0.001,0.001]);
% else %when inside the target area
%     P = diag([1,1,1,1,1,1,1,1]);
%     Q = diag([1,1,1,1,1,1,1,1]);
%     R = diag([1,1]);
% end

P = zeros(8);
Q = zeros(8);
R = eye(2);

A = param.A;
B = [param.B, zeros(8,length(u)-2)];
C = param.C;

% Constraints 
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);

% rectangle constraints
D = [ DRect(1,1) 0 DRect(1,2) 0 0 0 0 0; 
      DRect(2,1) 0 DRect(2,2) 0 0 0 0 0; 
      0          0 0          0 1 0 0 0; %limit on Theta
      0          0 0          0 0 0 1 0  %limit on Phi
    ];
  
ang_lim = 10*pi/180;
cl = [clRect; 
      -ang_lim; 
      -ang_lim
     ];
ch = [chRect; 
      ang_lim; 
      ang_lim
     ];
  
E = [ 1       , 0;   % -1 < u_x < 1
      0       , 1;   % -1 < u_y < 1
    ];
  
ul = [-1; 
      -1
     ];
uh = [ 1;  
       1
     ];

[Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh);

% Compute trajectory constraints matrices and vector
[DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N);

% Add temporary target constraints

if t + param.samples_max * param.Ts > param.Tf
    DD = [DD; [zeros(16,8*(N-1)), [eye(8); -eye(8)]]];
    EE = [EE; zeros(16,size(EE,2))];
    bb = [bb; 
          r(1:8) + param.tolerances.state(1:8)./2; 
          param.tolerances.state(1:8)./2 - r(1:8);
         ];
else
    selectPos = [ 1 0 0 0 0 0 0 0;
                  0 0 1 0 0 0 0 0 ];
    DD = [DD; [zeros(4,8*(N-1)), [selectPos; -selectPos]]];
    EE = [EE; zeros(4,size(EE,2))];
    bb = [bb; 
          r(1) + param.tolerances.state(1)/2; 
          r(3) + param.tolerances.state(3)/2; 
          param.tolerances.state(1)/2 - r(1);
          param.tolerances.state(3)/2 - r(3)
         ];
end


% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N);
[F,J,L] = genConstraintMatrices(DD,EE,Gamma,Phi,N,u_len);

% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);

% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
% [Lchol,p] = chol(H,'lower');
% Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
%u = genMPController(Linv,G,F,bb,J,L,x_MPC,r,length(u));
u = genMPController(H,G,F,bb,J,L,x_MPC,r,u_len,N,Phi,Gamma);


u = u(1:2);

end % End of myMPController







%% OTHER FUNCTIONS

