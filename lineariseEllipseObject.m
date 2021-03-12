function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b)
    alpha = ellipseEval(xg, yg, xc, yc, a, b);
    beta = 2 * (xg - xc) / (a^2);
    gamma = 2 *(yg - yc) / (b^2);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARow = [ -beta, 0, -gamma, 0, -beta*bx, 0, -beta*by, 0, 0, 0 ];
    bRow = alpha - beta * xg - gamma * yg + beta * ax + gamma * ay;
end

