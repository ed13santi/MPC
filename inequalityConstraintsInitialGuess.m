function [A, b] = inequalityConstraintsInitialGuess(N, ropeLen, rectConstraints, r, stateTol, inputTol)
    A = zeros(0, 8+10*N);
    b = zeros(0, 1);
    
    for i=1:N-1
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen);
        
        % combine matrices
        A_tmp = [A1;A2];
        A_tmp2 = [zeros(size(A_tmp,1), i*10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10-10)];
        A = [A; A_tmp2];
        b = [b; b1;b2];
    end
    
%     A_tmp = [ [ zeros(20, 10*(N-1)), [eye(10); -eye(10)], zeros(20, 8) ];
%               [ zeros(16, 10*N), [eye(8); -eye(8)] ] ];
%     b_tmp = [ r + stateTol; 
%               inputTol;
%               - r + stateTol;
%               inputTol;
%               r + stateTol; 
%               - r + stateTol ];
%           
%     A = [A; A_tmp];
%     b = [b; b_tmp];
end

