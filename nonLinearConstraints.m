function [c, ceq] = nonLinearConstraints(ellConstr, ellipses, dt, Ts, w)
    % inequality constraints for ellipses
    if ellConstr == true
        N = (length(w)-8)/10;
        c = [];
        for i=1:N-1
            x = w(10*i+1);
            y = w(10*i+3);
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
    
    % equality constraints due to dynamics of system
    ceq = w2x(w) - doStep(w, dt, Ts); % IF REACTIVATE THIS, NEED TO REMOVE
    %LINEAR MODEL CONSTRAINTS
end


%% run initial optimisation

