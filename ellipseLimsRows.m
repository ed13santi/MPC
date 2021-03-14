function [ARows, bRows] = ellipseLimsRows(ropeLen, ellipses, xg, yg, thetag, phig, extraDistance, nSlackVars)
    ellList = [];
    ellVals = [];
    ellObjVals = [];
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            ellList = [ ellList; ell ];
            % build list of the values of ellipseEval for cart
            ellVals = [ ellVals; ellipseEval(xg, yg, ell.xc, ell.yc, ell.a, ell.b) ];
            % build list of the values of ellipseEval for object
            ellObjVals = [ ellObjVals; ellipseEval(xg+ropeLen*sin(thetag), yg+ropeLen*sin(phig), ell.xc, ell.yc, ell.a, ell.b) ];
        end
    end
    [~,indexesCart] = sort(ellVals, 'ascend'); % indexes in order of most active due to cart
    [~,indexesObj] = sort(ellObjVals, 'ascend'); % indexes in order of most active due to object
    
    ARows = zeros(length(ellList)*2,10+nSlackVars);
    bRows = zeros(length(ellList)*2,1);
    
    ellipseSlackVars = nSlackVars - 4;
    
    % constraints of cart
    for i=1:ellipseSlackVars/2
        ell = ellList(indexesCart(i));
        [ARow1, bRow1] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b, extraDistance, ellipseSlackVars, i);
        size(ARows)
        size(ARow1)
        ARows = [ARows; ARow1];
        bRows = [bRows; bRow1];
    end 
    %constraints of object
    for i=1:ellipseSlackVars/2
        ell = ellList(indexesObj(i));
        [ARow2, bRow2] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, ell.xc, ell.yc, ell.a, ell.b, extraDistance, ellipseSlackVars, i);
        ARows = [ARows; ARow2];
        bRows = [bRows; bRow2];
    end 
end

