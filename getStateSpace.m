function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, Ts)
    xus = [x0; w0(9:end-8)];
    x = zeros(N*8, 1);
    u = zeros(N*2, 1);
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
    
    
%     x = w0(1:8);
%     u = w0(9:10);
%     [A, B, fref] = getLinearisation(x, u, 1, Ts, der, genA, genB);
       
    for i=1:N
       x = w0(i*10-9:i*10-2);
       u = w0(i*10-1:i*10);
       [A, B, fref] = getLinearisation(x, u, 1, Ts, der, genA, genB);
       Aeq(8+i*8-7:8+i*8, i*10-9:i*10-2) = A; 
       Aeq(8+i*8-7:8+i*8, i*10-1:i*10) = B;
       Aeq(8+i*8-7:8+i*8, i*10+1:i*10+8) = - eye(8);
       beq(8+i*8-7:8+i*8) = A*x + B*u - fref;
    end
end

