function [ARows, bRows] = rectLimRowsPath(rectConstraints)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    D = [ DRect(1,1) DRect(1,2); 
          DRect(2,1) DRect(2,2) ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
end

