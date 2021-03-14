function [ARows, bRows] = ellipseLimsRows(ropeLen, ellipses, xg, yg, thetag, phig, extraDistance)
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
    
    ARows = zeros(length(ellList)*2,10);
    bRows = zeros(length(ellList)*2,1);
    
    % constraints of cart
    n_constr = min(2, length(indexesCart));
    for ellInd=1:indexesCart(1:n_constr)
        ell = ellList(ellInd);
        [ARow1, bRow1] = lineariseEllipse(xg, yg, ell.xc, ell.yc, ell.a, ell.b, extraDistance);
        ARows = [ARows; ARow1];
        bRows = [bRows; bRow1];
    end 
    %constraints of object
    n_constr = min(2, length(indexesObj));
    for ellInd=1:indexesObj(1:n_constr)
        ell = ellList(ellInd);
        [ARow2, bRow2] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, ell.xc, ell.yc, ell.a, ell.b, extraDistance);
        ARows = [ARows; ARow2];
        bRows = [bRows; bRow2];
    end 
end

