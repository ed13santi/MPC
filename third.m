function [Aeq, beq] = third(A, B, x0, w0, genA, genB, der, N, radius, Ts)
    xus = [x0; radius; 0; w0(11:end-10)];
    x = zeros(N*10, 1);
    u = zeros(N*3, 1);
    Aeq = zeros(10+(10+2)*N, length(w0));
    beq = zeros(10+(10+2)*N, 1);
    
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; radius; 0];
       
    for i=1:N
       x = [xus(i*13-12:i*13-5); radius; 0];
       u = xus(i*13-2:i*13);
       [A, B, fref] = getLinearisation(x, u, 1, Ts, der, genA, genB);
       Aeq(10+i*12-11:10+i*12-2, i*13-12:i*13-3) = A; 
       Aeq(10+i*12-11:10+i*12-2, i*13-2:i*13) = B;
       Aeq(10+i*12-11:10+i*12-2, i*13+1:i*13+10) = - eye(10);
       beq(10+i*12-11:10+i*12-2) = A*x + B*u - fref; 
       beq(10+i*12-11:10+i*12-2) = 0;
       
       Aeq(10+i*12-1 :10+i*12, i*13+9:i*13+10) = eye(2); % r and r_dot constraints
       beq(10+i*12-1 :10+i*12) = [radius; 0]; % r and r_dot constraints
    end
end

