function [DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N)
% your code goes here
DD = kron(eye(N), Dt);
EE = kron(eye(N), Et);
bb = kron(ones(N,1), bt);
end


