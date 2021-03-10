function [ARows, bRows] = finalPositionRowsInitGuess(r, N)
    ARows = [ zeros(8, 10*N), eye(8) ];
    bRows = r;
end
