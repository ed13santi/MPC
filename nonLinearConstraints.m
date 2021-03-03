function [c, ceq] = nonLinearConstraints(dt, craneParams, w)
    c = [];
    ceq = w2x(w) - doStep(w2x_wave(w), w2u(w), dt, craneParams);
end




%% lower and upper bounds
