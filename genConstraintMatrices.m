function [F,J,L] = genConstraintMatrices(DD,EE,Gamma,Phi,N)

% your code goes here
n_states = size(DD,2) / N;

F = EE + DD * [eye(N*n_states), zeros(N*n_states, n_states)] * [zeros(n_states,size(Gamma,2)); Gamma];
J = - DD * [eye(N * n_states), zeros(N*n_states, n_states)] * [eye(size(Phi,2)); Phi];
L =  - DD * kron(ones(N,1), eye(n_states)) - J;

end



