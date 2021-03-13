function violate = violateRectConstr(rectConstr, x, y)
    [DRect,chRect,clRect] = rectCon(rectConstr);
    D = [ DRect(1,1) DRect(1,2); 
          DRect(2,1) DRect(2,2) ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
    vect = ARows * [x;y] - bRows;
    violate = any(vect > 0);
end

