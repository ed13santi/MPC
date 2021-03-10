function [ARows, bRows] = finalPositionRows(r, tolerances, N)
    selPos = [1 0 0 0 0 0 0 0; 
              0 0 1 0 0 0 0 0];
    ARows = [ zeros(4, 10*N), [selPos; -selPos] ];
    bRows = [ r(1,:) + tolerances(1,:)/2; 
              r(3,:) + tolerances(3,:)/2; 
              - r(1,:) + tolerances(1,:)/2; 
              - r(3,:) + tolerances(3,:)/2 ];
end




%% linear equality constraints
