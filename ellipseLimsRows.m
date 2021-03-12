function [ARows, bRows] = ellipseLimsRows(ropeLen, ellipses, xg, yg, thetag, phig)
    ARows = zeros(0,10);
    bRows = zeros(0,1);
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            [ARow1, bRow1] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b);
            [ARow2, bRow2] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, ell.xc, ell.yc, ell.a, ell.b);
            ARows = [ARows; ARow1; ARow2];
            bRows = [bRows; bRow1; bRow2];
        end
    end
end

