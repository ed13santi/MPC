function [A, B, x_next] = getLinearisation(x, u, Ns, Ts, F, Fs, Fv) %using Euler's method
    x_next = x;
    A = eye(10);
    B = zeros(10,3);
        
    Ti = Ts/Ns;
    for i=1:Ns
        x_next = x_next + Ti * F([x_next;u]);
        tmp = (eye(10) + Ti * Fs([x_next;u])) * [A, B] + [zeros(10), Ti * Fv([x_next;u])];
        A = tmp(:,1:10);
        B = tmp(:,11:end);
    end
end


