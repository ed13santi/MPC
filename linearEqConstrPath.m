function [Aeq, beq] = linearEqConstrPath(x_first, x_trgt, N)
    Aeq = [ [eye(2), zeros(2, 2*N)];
            [zeros(2, 2*N), eye(2)] ];
    beq = [x_first; x_trgt];
end

