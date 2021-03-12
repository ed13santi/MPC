function [c, ceq] = nonLinearConstraints(ropeLen, ellConstr, ellipses, dt, Ts, w)
    % inequality constraints for ellipses
    if ellConstr == true
        N = (length(w)-8)/10;
        c = [];
        for i=1:N-1
            x = w(10*i+1);
            y = w(10*i+3);
            angle_x = w(10*i+5);
            angle_y = w(10*i+7);
            x_p = x + ropeLen * sin(angle_x);
            y_p = y + ropeLen * sin(angle_y);
            for j=1:size(ellipses,1)
                for z=1:size(ellipses,2)
                    ell = ellipses{j,z};
                    c = [c; - ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b)];
                    c = [c; - ellipseEval(x_p, y_p, ell.xc, ell.yc, ell.a, ell.b)];
                end
            end
        end
    else
        c = [];
    end
    
    % equality constraints due to dynamics of system
    ceq = w2x(w) - doStep(w, dt, Ts); 
end


%% run initial optimisation

