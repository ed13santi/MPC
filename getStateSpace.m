function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, Ts, nSlackVars)
    secLen = 10 + nSlackVars;
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
       
    for i=1:N
       x = xus((i-1)*secLen+1:(i-1)*secLen+8);
       u = xus((i-1)*secLen+9+nSlackVars:(i-1)*secLen+10+nSlackVars);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, (i-1)*secLen+1:(i-1)*secLen+8) = A; 
       Aeq(8+i*8-7:8+i*8, (i-1)*secLen+9+nSlackVars:(i-1)*secLen+10+nSlackVars) = B;
       Aeq(8+i*8-7:8+i*8, i*secLen+1:i*secLen+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end
end


%% model linearisation
