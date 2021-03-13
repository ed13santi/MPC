function [c, ceq] = nonLinearConstraintsPath(ellConstr, ellipses, x_in, N)
    % inequality constraints for ellipses
    if ellConstr == true
        c = [];
        for i=1:N-1
            x = x_in(2*i+1);
            y = x_in(2*i+2);
            for j=1:size(ellipses,1)
                for z=1:size(ellipses,2)
                    ell = ellipses{j,z};
                    c = [c; - ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b)];
                end
            end
        end
    else
        c = [];
    end
    
    ceq = [];
end

%% Find path 2

