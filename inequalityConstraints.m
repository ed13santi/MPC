function [A, b] = inequalityConstraints(N, r, tolerances, ropeLen, rectConstraints, ellipses, w, n_final_pos_constrs)
    A = zeros(0, 8+10*N);
    b = zeros(0, 1);
    
    for i=1:N-1
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen);
        
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ellipses, w(10*i+1), w(10*i+3));
        
        
        A_tmp = [A1;A2;A3];
        A_tmp2 = [zeros(size(A_tmp,1), i*10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10-10)];
        A = [A; A_tmp2];
        b = [b; b1;b2;b3];
        
        if N - i < n_final_pos_constrs
            % final position constraint
            [A4, b4] = finalPositionRows(r, tolerances, i, N);
            A = [A; A4];
            b = [b; b4];
        end
    end
    
    % final position constraint
    [A_tmp, b_tmp] = finalPositionRows(r, tolerances, N, N);
    A = [A; A_tmp];
    b = [b; b_tmp];
end

