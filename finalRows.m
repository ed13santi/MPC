function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N, nSlackVars)
    segLen = 10 + nSlackVars;
    if i == N
        ARows = [ zeros(16, 10+segLen*(i-1)), [eye(8); -eye(8)], zeros(16, nSlackVars) ];
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        ARows = [ zeros(20, 10+segLen*(i-1)), [eye(10); -eye(10)], zeros(20, segLen*(N-i)+nSlackVars-2) ];
        bRows = [ r + tolerancesState/2;
                  tolerancesInput;
                  - r + tolerancesState/2;
                  tolerancesInput ];
    end
end




%% linear equality constraints
