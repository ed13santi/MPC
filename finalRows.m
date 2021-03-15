function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N, nSlackVars)
    segLen = 10 + nSlackVars;
    if i == N
        %         xuxXuxXuxXu                 x                  X
        ARows = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, nSlackVars) ]; 
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        %          xuxXuxXuxXu                 x                  XuxXuxX
        ARows1 = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, segLen*(N-i)+nSlackVars) ];
        %          xuxXuxXuxXuxX                         u                  xXuxX
        ARows2 = [ zeros(4, 18+nSlackVars+segLen*(i-1)), [eye(2); -eye(2)], zeros(4, segLen*(N-i)-2) ];
        ARows = [ARows1; ARows2];
        
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2;
                  tolerancesInput;
                  tolerancesInput ];
    end
end




%% linear equality constraints
