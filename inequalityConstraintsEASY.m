function [A, b] = inequalityConstraintsEASY(N, r, tolerances)
    A = zeros(8*2, 10+13*N);
    b = zeros(8*2, 1);
    
    A(end-15:end-8, end-9:end-2) = eye(8);
    A(end-7:end, end-9:end-2) = -eye(8);
    b(end-15:end-8) = r + tolerances;
    b(end-7:end) = tolerances - r;
end



%% linear equality constraints
