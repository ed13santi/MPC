function [A, b] = inequalityConstraints(N, r, tolerances)
    A = zeros(4*N+8*2, 8+10*N);
    b = zeros(4*N+8*2, 1);
    for i=1:N
        A(4*i-3,10*i-1) =  1; % u < 1
        A(4*i-2,10*i-1) = -1; % u > 1
        A(4*i-1,10*i) =  1; % u < 1
        A(4*i  ,10*i) = -1; % u > 1
        b(4*i-3:4*i) =  [1;1;1;1];
    end
    A(4*N+1:4*N+8 , 10*N+1:10*N+8) = eye(8);
    A(4*N+9:4*N+16, 10*N+1:10*N+8) = -eye(8);
    b(4*N+1:4*N+8)  = r + tolerances/2;
    b(4*N+9:4*N+16) = tolerances/2 - r;
end




%% linear equality constraints
