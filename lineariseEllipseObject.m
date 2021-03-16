function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b, extraDist, ellSlackVars, ellIndex)
    a_new = a + extraDist;
    b_new = b + extraDist; % pad around ellipse for robustness
    xg_p = xg + ropeLen * sin(thetag); 
    yg_p = yg + ropeLen * sin(phig); 
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a_new, b_new);
    beta =  2 * (xg_p - xc) / (a_new^2);
    gamma = 2 * (yg_p - yc) / (b_new^2);
    divBy = abs(beta) + abs(gamma);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARow = [ 0, 0, -beta/divBy, 0, -gamma/divBy, 0, -beta*bx/divBy, 0, -gamma*by/divBy, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    slackVars(2*ellIndex) = -1;
    ARow = [ ARow, slackVars ];
    ARow = [ ARow; [zeros(1, 14), slackVars] ];
    bRow = [ (alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay)/divBy;
             0 ];
end

