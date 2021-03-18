function [ARow, bRow] = lineariseEllipseObject(ropeLen, xg, yg, thetag, phig, xc, yc, a, b, extraDist, ellSlackVars, ellIndex)
    % hard
    xg_p = xg + ropeLen * sin(thetag); 
    yg_p = yg + ropeLen * sin(phig); 
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a, b);
    beta =  2 * (xg_p - xc) / (a^2);
    gamma = 2 * (yg_p - yc) / (b^2);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    ARowH = [ 0, 0, -beta, 0, -gamma, 0, -beta*bx, 0, -gamma*by, 0, 0, 0, 0, 0 ];
    slackVars = zeros(1, ellSlackVars);
    ARowH = [ ARowH, slackVars ];
    bRowH = alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay;
    
    % soft
    a_new = a + 2*extraDist*abs(a)/(abs(a)+abs(b));
    b_new = b + 2*extraDist*abs(b)/(abs(a)+abs(b)); % pad around ellipse for robustness
    alpha = ellipseEval(xg_p, yg_p, xc, yc, a_new, b_new);
    beta =  2 * (xg_p - xc) / (a_new^2);
    gamma = 2 * (yg_p - yc) / (b_new^2);
    divBy = abs(beta*(1+bx)) + abs(gamma*(1+by));
    ARowS = [ 0, 0, -beta/divBy, 0, -gamma/divBy, 0, -beta*bx/divBy, 0, -gamma*by/divBy, 0, 0, 0, 0, 0 ];
    slackVars(2*ellIndex) = -1;
    ARowS = [ ARowS, slackVars ];
    ARowS = [ ARowS; [zeros(1, 14), slackVars] ];
    bRowS = [ (alpha - beta * xg_p - gamma * yg_p + beta * ax + gamma * ay)/divBy;
             0 ];
         
    ARow = [ARowH;ARowS];
    bRow = [bRowH;bRowS];
end

