function [ARow, bRow] = lineariseEllipse(xg, yg, xc, yc, a, b)
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ARow = [-beta 0 -gamma 0 0 0 0 0 0 0];
    bRow = alpha - beta * xg - gamma * yg;
end

