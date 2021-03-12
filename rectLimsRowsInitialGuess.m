function [ARows, bRows] = rectLimsRowsInitialGuess(rectConstraints)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    D = [ DRect(1,1) 0 DRect(1,2) 0 0             0 0             0 0 0; 
          DRect(2,1) 0 DRect(2,2) 0 0             0 0             0 0 0 ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
end
