function [c, ceq] = nonLinearConstraints(ellipses, dt, Ts, w)
    % inequality constraints for ellipses
%     N = (length(w)-8)/10;
%     c = zeros(N*size(ellipses,1)*size(ellipses,2),1);
%     xVec = w2allX(w);
%     yVec = w2allY(w);
%     for i=1:N+1
%         for j=1:size(ellipses,1)
%             for z=1:size(ellipses,2)
%                 ell = ellipses{j,z};
%                 c(i) = - ellipseEval(xVec(i), yVec(i), ell.xc, ell.yc, ell.a, ell.b);
%             end
%         end
%     end
    c = [];
    
    % equality constraints due to dynamics of system
    ceq = w2x(w) - doStep(w, dt, Ts); % IF REACTIVATE THIS, NEED TO REMOVE
    %LINEAR MODEL CONSTRAINTS
end


%% run initial optimisation

