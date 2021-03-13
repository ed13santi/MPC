function f = pathObjFunc(x, N)
    T = [ [ zeros(2*N,2), eye(2*N) ];
          [ zeros(2,2*N), eye(2) ] ];
    p = eye(2*(N+1));
    H = p - T'*p' - p*T + T'*p*T;
    f = x' * H * x;
end

