function [A,B,C,D] = genModel(m,M,MR,r,g,Tx,Ty,Vx,Vy,Ts)
    % replace A,B,C,D with the correct values
    A=[ 0, 1,             0, 0,        0,                          0, 0,              0;
        0, -Tx/(M+MR),    0, 0,        g*m/(M+MR),                 0, 0,              0;
        0, 0,             0, 1,        0,                          0, 0,              0;
        0, 0,             0, -Ty/M,    0,                          0, g*m/M,          0;
        0, 0,             0, 0,        0,                          1, 0,              0;
        0, Tx/(r*(M+MR)), 0, 0,        -(g*(m+M+MR))/(r*(M+MR)),   0, 0,              0;
        0, 0,             0, 0,        0,                          0, 0,              1;
        0, 0,             0, Ty/(M*r), 0,                          0, -g*(m+M)/(M*r), 0 ];
    B=[ 0,              0;
        Vx/(M+MR),      0;
        0,              0;
        0,              Vy/M;
        0,              0;
        -Vx/(r*(M+MR)), 0; 
        0,              0; 
        0,              -Vy/(M*r) ];
    C=eye(8);
    D=zeros(8,2);
    
    % if Ts>0 then sample the model with a zero-order hold (piecewise constant) input, otherwise return a continuous-time model
    if Ts>0
        sys = ss(A,B,C,D);
        sysd = c2d(sys, Ts);
        A = sysd.A;
        B = sysd.B;
        C = sysd.C;
        D = sysd.D;
    end
end

