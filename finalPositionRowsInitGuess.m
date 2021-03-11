function [ARows, bRows] = finalPositionRowsInitGuess(r, N)
    ARows = [ zeros(8, 10*N), eye(8) ];
    bRows = r;
%     ARows = [ zeros(18, 10*(N-1)), eye(18) ];
%     bRows = [r;0;0;r];
end

