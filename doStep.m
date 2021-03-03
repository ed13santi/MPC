function x = doStep(x_wave, u, dt, craneParams) %why 3 u inputs???
    x = x_wave;
    for i = 1:length(x_wave)/10
        x_before = x_wave(i*10-9:i*10);
        odeFun = @(t,y) crane_nl_model_student([u(i*3-2:i*3)], y, craneParams);
        [~, y] = ode45(odeFun, [0 dt], x_before);
        x(i*10-9:i*10) = y(1);
    end
end

