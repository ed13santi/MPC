function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b)
    xg_p = xg + ropeLen * sin(thetag); 
    yg_p = yg + ropeLen * sin(phig); 
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a, b);
    beta =  2 * (xg_p - xc) / (a^2);
    gamma = 2 * (yg_p - yc) / (b^2);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARow = [ -beta, 0, -gamma, 0, -beta*bx, 0, -gamma*by, 0, 0, 0 ];
    bRow = alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay;
end

