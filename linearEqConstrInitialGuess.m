function [Aeq, beq] = linearEqConstrInitialGuess(x0, w0, genA, genB, der, N, Ts, r)
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8) = x0;
    
    % linear model constraints
    xus = [x0; w0(9:end-8)];
       
    for i=1:N
       x = xus(i*10-9:i*10-2);
       u = xus(i*10-1:i*10);
       [A, B, fref] = getLinearisation(x, u, 10, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, i*10-9:i*10-2) = A; 
       Aeq(8+i*8-7:8+i*8, i*10-1:i*10) = B;
       Aeq(8+i*8-7:8+i*8, i*10+1:i*10+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end

    % final position constraint
    [A_tmp, b_tmp] = finalPositionRowsInitGuess(r, N);
    Aeq = [Aeq; A_tmp];
    beq = [beq; b_tmp];
    
    
    
end

