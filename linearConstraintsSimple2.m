function [Aeq, beq] = linearConstraintsSimple2(x0, w0, Fs, Fv, F, N, radius, Ts)
    Aeq = zeros(8+8*N, 10*N+8);
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8,:) = x0;
    
    xus = [x0; w0(11:end-10)];
    for i=1:N
       x = xus(10*i-9:10*i-2);
       u = xus(10*i-1:10*i);
       [A, B, fref] = getLinearisation(x, u, 1, Ts, F, Fs, Fv);
       Aeq(8+i*8-7:8+i*8, i*10-9:i*10-2 )   = A; 
       Aeq(8+i*8-7:8+i*8, i*10-1:i*10   )   = B;
       Aeq(8+i*8-7:8+i*8, i*10+1:i*10+8 )   = -eye(8);
       beq(8+i*8-7:8+i*8)                   = A*x+B*u-fref;
    end
end


%% model linearisation
