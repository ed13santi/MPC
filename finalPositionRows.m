function [ARows, bRows] = finalPositionRows(r, tolerances, N)
    ARows = [ zeros(16, 10*N), [eye(8); -eye(8)] ];
    bRows = [r + tolerances/2; tolerances/2 - r];
end




%% linear equality constraints
