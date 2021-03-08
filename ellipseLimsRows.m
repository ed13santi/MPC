function [ARows, bRows] = ellipseLimsRows(ellipses, xg, yg)
    ARows = zeros(0,10);
    bRows = zeros(0,1);
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            [ARow, bRow] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b);
            ARows = [ARows; ARow];
            bRows = [bRows; bRow];
        end
    end
end

