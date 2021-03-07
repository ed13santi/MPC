function [genericA, genericB, der] = obtainJacs(cP)

    syms dx x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 ux uy uz
    g  = 9.81;
    m  = cP.m;
    Tl = cP.Tl;
    Ty = cP.Ty;
    Tx = cP.Tx;
    M  = cP.M;
    MR = cP.MR;
    Vx = cP.Vx;
    Vy = cP.Vy;
    Vl = cP.Vl;
    
    dx = sym(zeros(10, 1));

    dx(1, 1)  = x2;
    dx(2, 1)  = (cos(x5) .* cos(x7) .^ 2 .* sin(x5) .* g .* m + sin(x5) .* (Tl .* x10 - Vl * uz) .* cos(x7) - Tx .* x2 + Vx * ux) ./ (M + MR);
    dx(3, 1)  = x4;
    dx(4, 1)  = ((m .* g .* cos(x5) .* cos(x7) + Tl .* x10 - Vl * uz) .* sin(x7) - Ty .* x4 + Vy * uy) ./ M;
    dx(5, 1)  = x6;  
    dx(6, 1)  = (-cos(x5) .^ 2 .* cos(x7) .^ 2 .* sin(x5) .* g .* m + (-sin(x5) .* (Tl .* x10 - Vl * uz) .* cos(x5) - 0.2e1 .* x6 .* x10 .* (M + MR)) .* cos(x7) + (Tx .* x2 - Vx * ux) .* cos(x5) + 0.2e1 .* (M + MR) .* (sin(x7) .* x6 .* x8 .* x9 - sin(x5) .* g ./ 0.2e1)) ./ x9 ./ cos(x7) ./ (M + MR);
    dx(7, 1)  = x8;
    dx(8, 1)  = (-cos(x5) .* g .* sin(x7) .* m .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 2 + ((-M .* (Tl .* x10 - Vl * uz) .* cos(x5) .^ 2 - M .^ 2 .* x6 .^ 2 .* x9 - x6 .^ 2 .* x9 .* MR .* M - MR .* (Tl .* x10 - Vl * uz)) .* sin(x7) - (-Ty .* x4 + Vy * uy) .* (M + MR)) .* cos(x7) - 0.2e1 .* ((g .* (M + MR) .* cos(x5) ./ 0.2e1 + sin(x5) .* (Tx .* x2 - Vx * ux) ./ 0.2e1) .* sin(x7) + x10 .* x8 .* (M + MR)) .* M) ./ x9 ./ M ./ (M + MR);
    dx(9, 1)  = x10;
    dx(10, 1) = (cos(x5) .* g .* m .^ 2 .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 3 + m .* (M .* (Tl .* x10 - Vl * uz) .* cos(x5) .^ 2 + M .^ 2 .* x6 .^ 2 .* x9 + x6 .^ 2 .* x9 .* MR .* M + MR .* (Tl .* x10 - Vl * uz)) .* cos(x7) .^ 2 + (-g .* m .* (M + MR) .* cos(x5) + M .* sin(x5) .* (Tx .* x2 - Vx * ux)) .* m .* cos(x7) + (M + MR) .* (-m .* (-Ty .* x4 + Vy * uy) .* sin(x7) + (M .* x8 .^ 2 .* x9 - Tl .* x10 + Vl * uz) .* m - M .* (Tl .* x10 - Vl * uz))) ./ m ./ M ./ (M + MR);
   
    jacA = jacobian(dx, [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10]);
    jacB = jacobian(dx, [ux uy uz]);

    genericA = @(w) double(subs(jacA, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, ux, uy, uz], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
    genericB = @(w) double(subs(jacB, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, ux, uy, uz], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
    der      = @(w) double(subs(dx  , [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, ux, uy, uz], [w(1), w(2), w(3), w(4), w(5), w(6), w(7), w(8), w(9), w(10), w(11), w(12), w(13)]));
end


