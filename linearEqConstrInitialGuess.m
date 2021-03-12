function [Aeq, beq] = linearEqConstrInitialGuess(x0, w0, genA, genB, der, N, Ts, r)
    xus = [x0; w0(9:end-8)];
    Aeq = zeros(8+8*N, length(w0));
    beq = zeros(8+8*N, 1);
    
    Aeq(1:8,1:8) = eye(8);
    beq(1:8) = x0;

    % final position constraint
    [A_tmp, b_tmp] = finalPositionRowsInitGuess(r, N);
    Aeq = [Aeq; A_tmp];
    beq = [beq; b_tmp];
end

