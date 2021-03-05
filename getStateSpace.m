function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, der, N, radius, Ts)
    xus = [x0; radius; 0; w0(11:end-10)];
    x = zeros(N*10, 1);
    u = zeros(N*3, 1);
    Aeq = zeros(10+(10+2)*N, length(w0));
    beq = zeros(10+(10+2)*N, 1);
    
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; radius; 0];
    for i=1:N
       x = xus(13*i-12:13*i-3);
       u = xus(13*i-2:13*i);
       funcInp = [x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), u(1), u(2), u(3)];
       A = genA(funcInp);
       B = genB(funcInp);
       Aeq(10+i*12-11:10+i*12-2, i*13-12:i*13-3) = eye(length(x)) + Ts*A/2; 
       Aeq(10+i*12-11:10+i*12-2, i*13-2:i*13) = Ts*B;
       Aeq(10+i*12-11:10+i*12-2, i*13+1:i*13+10) = Ts*A/2 - eye(length(x));
       Aeq(10+i*12-1 :10+i*12, i*13+9:i*13+10) = eye(2); % r and r_dot constraints
       beq(10+12*i-11:10+12*i-2) = - Ts * (der(funcInp) - A*x - B*u); % b (signs reversed cuz on other side of eqn) MUST USE the continuous A B
       beq(10+12*i-1 :10+12*i) = [radius; 0]; % r and r_dot constraints
    end
end

