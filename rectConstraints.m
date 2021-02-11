function [mat, ch, cl] = rectConstraints(rect)
    A = rect(1,:);
    B = rect(2,:);
    C = rect(3,:);
    D = rect(4,:);
    [a1,c1] = lineParams(A,D);
    [a2,c2] = lineParams(A,B);
    [a3,c3] = lineParams(B,C);
    [a4,c4] = lineParams(D,C);
    
    mat  = [ a1 , 1;
             a2 , 1 ];% CHECK THIS MIGHT BE WRONG!!!
         
    ch = [ c3; c2 ];
    cl = [ c1; c4 ];
end




