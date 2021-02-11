function [H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N)
%% cost function matrices
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x'Qx
% R is the stage cost weight on the inputs, i.e. u'Ru
% P is the terminal weight on the final state

% Your code goes here
extendedGamma = [zeros(size(Phi,2), size(Gamma,2)); Gamma];
bigQ = kron(eye(N+1), Q);
bigQ(end-size(P,1)+1:end, end-size(P,2)+1:end) = P;
bigR = kron(eye(N),R);
H = (extendedGamma'*bigQ*extendedGamma + bigR) * 2;

extendedPhi = [eye(size(Phi,2)); Phi];
G = (extendedGamma'*bigQ'*extendedPhi) * 2;

end
