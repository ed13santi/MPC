function [ARows, bRows] = finalRows(r, tolerancesState, tolerancesInput, i, N)
    if i == N
        ARows = [ zeros(16, 10*i), [eye(8); -eye(8)] ];
        bRows = [ r + tolerancesState/2;
                  - r + tolerancesState/2 ];
    else
        ARows = [ zeros(20, 10*i), [eye(10); -eye(10)], zeros(20, 10*N-10*i-2) ];
        bRows = [ r + tolerancesState/2;
                  tolerancesInput;
                  - r + tolerancesState/2;
                  tolerancesInput ];
    end
end




%% linear equality constraints
