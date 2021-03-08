function x = doStep(w, dt, Ts) 
    N = (length(w)-8)/10;
    x = zeros(8*N,1);
    for i = 1:N
        x_before = w(i*10-9:i*10-2);
        odeFun = @(t,y) dt([y; w(i*10-1:i*10)]);
        [~, y] = ode45(odeFun, [0 Ts], x_before);
        x(i*8-7:i*8) = y(end,:);
    end
end

