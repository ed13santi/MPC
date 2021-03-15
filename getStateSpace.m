function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, Ts, nSlackVars)
    secLen = 10 + nSlackVars;
    xus = [x0; w0(9:end-8)]; % xuxXuxXuxXuxX
    Aeq = zeros(8+8*N, length(w0)); 
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
    
    x = x0;
    u = w0(9:10);
    [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
    Aeq(9:16, 1:8) = A; 
    Aeq(9:16, 9:10) = B;
    Aeq(9:16, 11:18) = - eye(8);
    beq(9:16) = A*x + B*u - fref;
       
    for i=2:N
       x = xus(10+(i-2)*secLen+1:10+(i-2)*secLen+8); 
       u = xus(10+(i-2)*secLen+9+nSlackVars:10+(i-2)*secLen+10+nSlackVars);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, 10+(i-2)*secLen+1:10+(i-2)*secLen+8) = A; 
       Aeq(8+i*8-7:8+i*8, 10+(i-2)*secLen+9+nSlackVars:10+(i-2)*secLen+10+nSlackVars) = B;
       Aeq(8+i*8-7:8+i*8, 10+(i-1)*secLen+1:10+(i-1)*secLen+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end
end


%% model linearisation
