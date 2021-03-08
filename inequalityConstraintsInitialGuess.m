function [A, b] = inequalityConstraintsInitialGuess(N, r, tolerances, ropeLen, rectConstraints, ellipses, w)
    n_ellipses = size(ellipses,1) * size(ellipses,2);
    n_each = 12 + n_ellipses;
    A = zeros(n_each*N+8*2, 8+10*N);
    b = zeros(n_each*N+8*2, 1);
    
    for i=1:N
        % input physical limits
        A(n_each*i-11,10*i-1) =  1; % u < 1
        A(n_each*i-10,10*i-1) = -1; % u > 1
        A(n_each*i-9 ,10*i)   =  1; % u < 1
        A(n_each*i-8 ,10*i)   = -1; % u > 1
        b(n_each*i-11:n_each*i-8) =  [1;1;1;1];
        
        % rectangle constraints
        [DRect,chRect,clRect] = rectCon(rectConstraints);
        D = [ DRect(1,1) 0 DRect(1,2) 0 0                  0 0                  0; 
              DRect(2,1) 0 DRect(2,2) 0 0                  0 0                  0;  
              DRect(1,1) 0 DRect(1,2) 0 ropeLen*DRect(1,1) 0 ropeLen*DRect(1,2) 0;
              DRect(2,1) 0 DRect(2,2) 0 ropeLen*DRect(2,1) 0 ropeLen*DRect(2,2) 0 ];
        A(n_each*i-7-n_ellipses:n_each*i-4-n_ellipses,10*i-9:10*i-2) = D;
        A(n_each*i-3-n_ellipses:n_each*i-n_ellipses,10*i-9:10*i-2) = -D;
        b(n_each*i-7-n_ellipses:n_each*i-4-n_ellipses) = [chRect; chRect];
        b(n_each*i-3-n_ellipses:n_each*i-n_ellipses) = [-clRect; -clRect];
        
    end
    
    % final position constraint
    A(12*N+1:12*N+8 , 10*N+1:10*N+8) = eye(8);
    A(12*N+9:12*N+16, 10*N+1:10*N+8) = -eye(8);
    b(12*N+1:12*N+8)  = r + tolerances/2;
    b(12*N+9:12*N+16) = tolerances/2 - r;
end
