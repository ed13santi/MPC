function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b, extraDist, ellSlackVars, ellIndex)
    a_new = a + 2*extraDist*abs(a)/(abs(a)+abs(b));
    b_new = b + 2*extraDist*abs(b)/(abs(a)+abs(b)); % pad around ellipse for robustness
    alpha = ellipseEval(xg, yg, xc, yc, a_new, b_new);
    beta = 2 * (xg - xc) / (a_new^2);
    gamma = 2 *(yg - yc) / (b_new^2);
    divBy = abs(beta) + abs(gamma);
    ARow = [ 0, 0, -beta/divBy, 0, -gamma/divBy, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    slackVars(:, 2*ellIndex-1) = -1;
    ARow = [ ARow, slackVars ];
    ARow = [ ARow; [zeros(1, 14), slackVars] ];
    bRow = [ (alpha - beta * xg - gamma * yg)/divBy;
             0 ];
%     bRow = alpha - beta * xg - gamma * yg;
end

