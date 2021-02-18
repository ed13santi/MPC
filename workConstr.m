function [c,ceq] = workConstr(u, x0, N, Phi, Gamma)
    n_states = length(x0);
    extrXDot = kron(eye(N), [0 1 0 0 0 0 0 0]);
    extrYDot = kron(eye(N), [0 0 0 1 0 0 0 0]);
    extrUx =      kron(eye(N), [1 0 0 0]);
    extrUy =      kron(eye(N), [0 1 0 0]);
    extrLambdaX = kron(eye(N), [0 0 1 0]);
    extrLambdaY = kron(eye(N), [0 0 0 1]);
    x_wave = [eye(N*n_states), zeros(N*n_states, n_states)] * ([eye(size(Phi,2)); Phi] * x0 + [zeros(n_states,size(Gamma,2)); Gamma] * u);
    c = [ (extrXDot * x_wave) .* (extrUx * u) - extrLambdaX * u;
          (extrYDot * x_wave) .* (extrUy * u) - extrLambdaY * u ];
    c = [];
    ceq = [];
end
