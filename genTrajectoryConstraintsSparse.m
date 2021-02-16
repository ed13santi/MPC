function [D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N)
% your code goes here
block = [Dt, Et];
D = blkdiag(kron(eye(N), block), Dt);
d = kron(ones(N+1,1), bt);
end


