function [A, b] = inequalityConstraints(N, r, tolState, tolInput, ropeLen, rectConstraints, ellipses, w, n_final)
    if n_final < N + 1
        A = zeros(0, 8+10*N);
        b = zeros(0, 1);
    else
        A = [ zeros(4,8), [eye(2); -eye(2)], zeros(4,10*(N-1)+8) ];
        b = [ tolInput; tolInput ];
    end
    
    for i=1:N-1
        % input physical limits
        [A1, b1] = physicalLims;
        
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen);
        
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ropeLen, ellipses, w(10*i+1), w(10*i+3), w(10*i+5), w(10*i+7));
        
        
        A_tmp = [A1;A2;A3];
        A_tmp2 = [zeros(size(A_tmp,1), i*10), A_tmp, zeros(size(A_tmp,1), 8+10*N-i*10-10)];
        A = [A; A_tmp2];
        b = [b; b1;b2;b3];
        
        if N - i < n_final
            % final state/input constraint
            [A4, b4] = finalRows(r, tolState, tolInput, i, N);
            A = [A; A4];
            b = [b; b4];
        end
    end
    
    if n_final > 0
        % final state/input constraint
        [A_tmp, b_tmp] = finalRows(r, tolState, tolInput, N, N);
        A = [A; A_tmp];
        b = [b; b_tmp];
    end
end

