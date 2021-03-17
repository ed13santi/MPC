function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N, nSlackVars)
    segLen = 10 + nSlackVars;
    
    slacks = [ -1 0 0 0 0;
               0 -1 0 0 0;
               -1 0 0 0 0;
               0 -1 0 0 0;
               0 0 -1 0 0;
               0 0 0 -1 0;
               0 0 -1 0 0;
               0 0 0 -1 0];
    slacksU = [ 0 0 0 0 -1;
                0 0 0 0 -1];
               
    if i == N
        
        %         xuxXuxXuxXu                 x                  X
        ARows = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, nSlackVars), [slacks;slacks] ]; 
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        %          xuxXuxXuxXu                 x                  XuxXuxX
        ARows1 = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, segLen*(N-i)+nSlackVars),[slacks;slacks] ];
        %          xuxXuxXuxXuxX                         u                  xXuxX
        ARows2 = [ zeros(4, 18+nSlackVars+segLen*(i-1)), [eye(2); -eye(2)], zeros(4, segLen*(N-i)-2),[slacksU;slacksU] ];
        ARows = [ARows1; ARows2];
        
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2;
                  tolerancesInput;
                  tolerancesInput ];
    end
end




%% linear equality constraints
