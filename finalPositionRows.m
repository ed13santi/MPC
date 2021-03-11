function [ARows, bRows] = finalPositionRows(r, tolerances, i, N)
    selPos = [1 0 0 0 0 0 0 0; 
              0 0 1 0 0 0 0 0];
    ARows = [ zeros(4, 10*i), [selPos; -selPos], zeros(4, 10*N-10*i) ];
    bRows = [ r(1,:) + tolerances(1,:)/2; 
              r(3,:) + tolerances(3,:)/2; 
              - r(1,:) + tolerances(1,:)/2; 
              - r(3,:) + tolerances(3,:)/2 ];
end

