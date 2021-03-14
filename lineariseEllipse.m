function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b, extraDistance, ellSlackVars, ellIndex)
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ARow = [ 0, 0, -beta, 0, -gamma, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    slackVars(:, 2*ellIndex-1) = -1;
    ARow = [ ARow, slackVars ];
    ARow = [ ARow; [zeros(1, 14), -slackVars] ];
    bRow = [ 0;
             - (alpha - beta * xg - gamma * yg - extraDistance) ];
end

