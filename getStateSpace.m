function [Aeq, beq] = getStateSpace(x0, w0, genA, genB, N, r, Ts)
    xus = [x0; r; 0; w0(11:end-10)];
    x = zeros(N*10, 1);
    u = zeros(N*3, 1);
    Aeq = zeros(10+(10+2)*N, length(w0));
    beq = zeros(10+(10+2)*N, 1);
    
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; r; 0];
    for i=1:N
       x = xus(13*i-12:13*i-3);
       u = xus(13*i-2:13*i);
       funcInp = [x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), r, 0, u(1), u(2), u(3)];
       A = genA(funcInp);
       B = genB(funcInp);
       beq(12*i-11:12*i-2) = A*x + B*u; % b (signs reversed cuz on other side of eqn) MUST USE the continuous A B
       sys = ss(A,B,eye(10),zeros(10,3));
       sysd = c2d(sys, Ts);
       A = sysd.A;
       B = sysd.B;
       Aeq(i*12-11:i*12-2, i*13-12:i*13-3) = A; % Ax_k + Bu_k - x_k+1 = b_k
       Aeq(i*12-11:i*12-2, i*13-2:i*13) = B;
       Aeq(i*12-11:i*12-2, i*13+1:i*13+10) = - eye(10);
       Aeq(i*12-1:i*12, i*13+9:i*13+10) = eye(2); % r and r_dot constraints
       beq(12*i-1:12*i) = [r; 0]; % r and r_dot constraints
    end
    
end


%% model linearisation
