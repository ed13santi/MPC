function [A, b] = inequalityConstraintsPath(N, rectConstraints)
    A = zeros(0, 2+2*N);
    b = zeros(0, 1);
    
    for i=1:N-1        
        % rectangle constraints
        [A2, b2] = rectLimRowsPath(rectConstraints);
        
        % combine matrices
        A_tmp = [zeros(size(A2,1), i*2), A2, zeros(size(A2,1), 2*N-i*2)];
        A = [A; A_tmp];
        b = [b; b2];
    end
end

