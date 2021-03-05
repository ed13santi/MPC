function [Aeq, beq] = linearConstraintsSimple(A, B, x0, N, r, trgt)
    Aeq = zeros(10+10*N, 13*N+10);
    beq = zeros(10+10*N, 1);
        
    Aeq(1:10,1:10) = eye(10);
    beq(1:10,:) = [x0; r; 0];
    for i=1:N
       Aeq(10+i*10-9:10+i*10-2, i*13-12:i*13-3) = [A, zeros(8,2)];
       Aeq(10+i*10-9:10+i*10-2, i*13-2 :i*13) = [B, zeros(size(B,1),1)];
       Aeq(10+i*10-9:10+i*10-2, i*13+1 :i*13+10) = [- eye(size(A,1)), zeros(size(A,1),2)];
       beq(10+i*10-9:10+i*10-2) = zeros(8,1); % b (signs reversed cuz on other side of eqn) MUST USE the continuous A B
       
       Aeq(10+i*10-1:10+10*i  , i*13+9 :i*13+10) = eye(2); % r and r_dot constraints
       beq(10+i*10-1:10+10*i) = [r; 0]; % r and r_dot constraints
    end
end


%% model linearisation
