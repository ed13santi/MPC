function out = workOfStep(xuxVec)
    x = xuxVec(1:10);
    u = xuxVec(11:13);
    x_next = xuxVec(14:23);
    workX = max(0, u(1) * (x(2) + x_next(2))) / 2; % /2 is not really needed
    workY = max(0, u(2) * (x(4) + x_next(4))) / 2; % /2 is not really needed
    out = workX + workY;
end





%% linear equality constraints
