function [A, b, indexesToShift] = inequalityConstraints(N, r, tolState, tolInput, ropeLen, rectConstraints, ellipses, w, n_final, extraDistanceEllipses, extraDistRect, nSlackVars)
    secLen = 10 + nSlackVars;
    
    indexesToShift = [];

    if n_final < N + 1
        A = zeros(0, 8+secLen*N+5);
        b = zeros(0, 1);
    else
        A = [ zeros(4,8), [eye(2); -eye(2)], zeros(4,secLen*N-2+5) ];
        b = [ tolInput; tolInput ];
    end
    
    for i=1:N
        x = w(10+(i-1)*secLen+1);
        y = w(10+(i-1)*secLen+3);
        theta = w(10+(i-1)*secLen+5);
        phi = w(10+(i-1)*secLen+7);
        
        
        % input physical limits
        [A1, b1] = physicalLimsWithSlack(nSlackVars);
        % rectangle constraints
        [A2, b2] = rectLimsRows(rectConstraints, ropeLen, theta, phi, extraDistRect, nSlackVars);
        % ellipse constraints
        [A3, b3] = ellipseLimsRows(ropeLen, ellipses, x, y, theta, phi, extraDistanceEllipses, nSlackVars);
        
        A_tmp = [A1;A2;A3];
        %                              xuxX             uxX      uxXuxX
        A_tmp2 = [zeros(size(A_tmp,1), 8+(i-1)*secLen), A_tmp, zeros(size(A_tmp,1), secLen*(N-i)+5)];
        A = [A; A_tmp2];
        b = [b;b1;b2;b3];
        
        if i>=2
            indexesToShift = [indexesToShift, size(A,1)+1-size(A_tmp2,1):size(A,1)];
        end
        
        if N - i < n_final
            % final state/input constraint
            [A4, b4] = finalRows(r, tolState, tolInput, i, N, nSlackVars);
            A = [A; A4];
            b = [b; b4];
            if i == 1
                indexesToShift = [indexesToShift, size(A,1)-3:size(A,1)];
            end
            if i>1
                indexesToShift = [indexesToShift, size(A,1)+1-size(A4,1):size(A,1)];
            end
        end
    end
    
%     if n_final > 0
%         % final state/input constraint
%         [A_tmp, b_tmp] = finalRows(r, tolState, tolInput, N, N, nSlackVars);
%         A = [A; A_tmp];
%         b = [b; b_tmp];
%     end
    
    % lambda > 0
    A = [ A; [zeros(5,size(A,2)-5), -eye(5)] ];
    b = [ b; zeros(5,1) ];
end

