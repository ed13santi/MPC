function u = genMPController(H,G,F,bb,J,L,x,xTarget,m,N,Phi,Gamma)
% H       - quadratic term in the cost function (Linv if using mpcActiveSetSolver).
% G       - matrix in the linear term in the cost function.
% F       - LHS of the inequalities term.
% bb      - RHS of the inequalities term.
% J       - RHS of inequalities term.
% L       - RHS of inequalities term. Use this to shift constraints around target point
% x       - current state
% xTarget - target equilibrium point.
% m       - Number of inputs.
% iA      - active inequalities, see doc mpcqpsolver
%
% u is the first input vector u_0 in the sequence [u_0; u_1; ...; u_{N-1}]; 
% In other words, u is the value of the receding horizon control law
% evaluated with the current state x0 and target xTarget

% Please read the documentation on mpcActiveSetSolver to understand how it is
% suppose to be used. Use iA and iA1 to pass the active inequality vector 

opt.MaxIterations = 200;
opt.IntegrityChecks = false;%% for code generation
opt.ConstraintTolerance = 1e-3;
opt.DataType = 'double';
opt.UseHessianAsInput = false;
%% your code starts here
linTerm = G * (x - xTarget);
if size(J,1) == 0
    rightIneqConstr = [];
else
    rightIneqConstr = bb + J*x + L*xTarget;
end
% persistent iA
% if isempty(iA)
%    iA = [];
% end
% [U,~,iA,~] = mpcActiveSetSolver(H, linTerm, F, rightIneqConstr, [], zeros(0,1), iA, opt);
% 
persistent u0
if isempty(u0)
    u0 = zeros(m*N,1);
end
%options =  optimset('Display', 'on','UseHessianAsInput','False');
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off')
U = quadprog(H, linTerm, F, rightIneqConstr, [], zeros(0,1), [], [], u0, options);
%objFunc = @(u) 0.5*u'*H*u + linTerm'*u;
%nonl = @(u) workConstr(u, x, N, Phi, Gamma);
%options = optimoptions('fmincon', 'Algorithm', 'interior-point')
%U = fmincon(objFunc, u0, F, rightIneqConstr, [], [], [], [], nonl, options);
%% your remaining code here
u = U(1:2);

u0 = U;
end



