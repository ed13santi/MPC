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
    w0 = zeros(m*N+8*(N+1)+1,1);
end
%options =  optimset('Display', 'on','UseHessianAsInput','False');
options = optimoptions('fmincon', 'Algorithm', 'active-set')
%W = quadprog(H, f, D, d, G, g, [], [], w0, options);
nonLinConstr = @(w) workConstr(w, m, N);
func = @(w) 0.5 * w' * H * w + f' * w;
W = fmincon(func, w0, D, d, G, g, [], [], nonLinConstr, options)
%% your remaining code here
u = W(9:10);
W0 = W;
end

