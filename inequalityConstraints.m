function [A, b] = inequalityConstraints(N, r, tolerances, ropeLen, rectConstraints)
    A = zeros(12*N+8*2, 8+10*N);
    b = zeros(12*N+8*2, 1);
    
    for i=1:N
        % input physical limits
        A(12*i-11,10*i-1) =  1; % u < 1
        A(12*i-10,10*i-1) = -1; % u > 1
        A(12*i-9 ,10*i)   =  1; % u < 1
        A(12*i-8 ,10*i)   = -1; % u > 1
        b(12*i-11:12*i-8) =  [1;1;1;1];
        
        % rectangle constraints
        [DRect,chRect,clRect] = rectCon(rectConstraints);
        D = [ DRect(1,1) 0 DRect(1,2) 0 0                  0 0                  0; 
              DRect(2,1) 0 DRect(2,2) 0 0                  0 0                  0;  
              DRect(1,1) 0 DRect(1,2) 0 ropeLen*DRect(1,1) 0 ropeLen*DRect(1,2) 0;
              DRect(2,1) 0 DRect(2,2) 0 ropeLen*DRect(2,1) 0 ropeLen*DRect(2,2) 0 ];
        A(12*i-7:12*i-4,10*i-9:10*i-2) = D;
        A(12*i-3:12*i,10*i-9:10*i-2) = -D;
        b(12*i-7:12*i-4) = [chRect; chRect];
        b(12*i-3:12*i) = [-clRect; -clRect];
    end
    
    % final position constraint
    A(12*N+1:12*N+8 , 10*N+1:10*N+8) = eye(8);
    A(12*N+9:12*N+16, 10*N+1:10*N+8) = -eye(8);
    b(12*N+1:12*N+8)  = r + tolerances/2;
    b(12*N+9:12*N+16) = tolerances/2 - r;
end

