function [ARows, bRows] = rectLimsRows(rectConstraints, ropeLen, thetag, phig, extraDistRect, nSlackVars)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    ax = ropeLen * (sin(thetag) - thetag*cos(thetag));
    ay = ropeLen * (sin(phig) - phig*cos(phig));
    bx = ropeLen * cos(thetag);
    by = ropeLen * cos(phig);
    D = [ 0 0 DRect(1,1) 0 DRect(1,2) 0 0             0 0             0; 
          0 0 DRect(2,1) 0 DRect(2,2) 0 0             0 0             0; 
          0 0 DRect(1,1) 0 DRect(1,2) 0 bx*DRect(1,1) 0 by*DRect(1,2) 0;
          0 0 DRect(2,1) 0 DRect(2,2) 0 by*DRect(2,1) 0 by*DRect(2,2) 0 ];
    ARows = [ [ D,           -eye(4), zeros(4,max(0, nSlackVars-4)) ]; 
              [ zeros(4,10), -eye(4), zeros(4,max(0, nSlackVars-4)) ];
              [ -D,          -eye(4), zeros(4,max(0, nSlackVars-4)) ]; 
              [ zeros(4,10), -eye(4), zeros(4,max(0, nSlackVars-4)) ] ];
    offset = [ - ax * DRect(1,1) - ay * DRect(1,2);
               - ax * DRect(2,1) - ay * DRect(2,2) ];
    bRows = [ - extraDistRect * ones(4,1);
              - chRect; 
              - chRect - offset; 
              - extraDistRect * ones(4,1);
              clRect;
              clRect + offset ];
end

