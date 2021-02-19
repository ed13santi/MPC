function [Gamma,Phi] = genPred(A,B,N)
    % GENPREDICTION  [Gamma,Phi] = genPrediction(A,B,N). 
    % A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
    % N is the horizon length. 
    % Your code is suppose to work for any linear system, not just the gantry crane. 
    
    % Write your code here
    n_states = size(A,1);
    n_inputs = size(B,2);
    
    bigA = eye(n_states*(N+1));
    bigA(n_states+1:end,1:end-n_states) = bigA(n_states+1:end,1:end-n_states) + kron(eye(N),-A);
    
    bigB = zeros(n_states*(N+1),n_inputs*N);
    bigB(n_states+1:end,1:end) = kron(eye(N),B);
    
    extender = [eye(n_states); zeros(n_states*N,n_states)];
    
    Phi = bigA\extender;
    Phi = Phi(n_states+1:end,1:end);
    Gamma = bigA\bigB;
    Gamma = Gamma(n_states+1:end,1:end);
end


