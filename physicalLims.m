function [ARows, bRows] = physicalLims()
    ARows = zeros(4,10);
    ARows(1,9) =  1; % u < 1
    ARows(2,9) = -1; % u > 1
    ARows(3,10) =  1; % u < 1
    ARows(4,10) = -1; % u > 1
    bRows = [1;1;1;1];
end

