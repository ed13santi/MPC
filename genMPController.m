function u = genMPController(H,G,F,bb,J,L,x,xTarget,m,N,Phi,Gamma,n_constr)

linTerm = G * (x - xTarget);
if size(J,1) == 0
    rightIneqConstr = [];
else
    rightIneqConstr = bb + J*x + L*xTarget;
end

persistent u0
if isempty(u0)
    u0 = zeros(m*N,1);
end

diff = length(u0) - m*N;
if diff ~= 0
    u0 = u0(1+diff:end);
end

persistent iA
if isempty(iA)
   iA = false(size(rightIneqConstr));
end

opt.MaxIterations = 200;
opt.IntegrityChecks = false;%% for code generation
opt.ConstraintTolerance = 1e-3;
opt.DataType = 'double';
opt.UseHessianAsInput = true;

[U,~,iA,~] = mpcActiveSetSolver(H, linTerm, F, rightIneqConstr, [], zeros(0,1), iA, opt);

iA = [iA(1+n_constr:end); false(n_constr,1)];


u = U(1:2);
u0 = U;
end



