function violate = violateEllConstr(ellipses, x, y)
    violate = false;
    for j=1:size(ellipses,1)
        for z=1:size(ellipses,2)
            ell = ellipses{j,z};
            if ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b) <= 0
                violate = true;
            end
        end
    end
end

