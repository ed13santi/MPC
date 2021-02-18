function [D,d] = genTrajectoryConstraintsSparse(Dt,Et,bt,N)
block = [Dt, Et];
D = kron(eye(N), block);
d = kron(ones(N,1), bt);
end
