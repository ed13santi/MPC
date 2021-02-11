clear all

function [mat, ch, cl] = rectConstraints(rect)
    [A; B; C; D] = rect;
    [a1,c1] = lineParams(A,D);
    [a2,c2] = lineParams(A,B);
    [a3,c3] = lineParams(B,C);
    [a4,c4] = lineParams(D,C);
    
    mat  = [ a1 , 1;
             a2 , 1 ];
         
    ch = [ c3; c4 ];
    cl = [ c1; c2 ];
end