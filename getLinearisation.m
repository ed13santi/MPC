function [A, B, x_next] = getLinearisation(x, u, Ns, Ts, F, Fs, Fv) %using Euler's method
    x_next = x;
    A = eye(8);
    B = zeros(8,2);
        
    Ti = Ts/Ns;
    for i=1:Ns
        x_next = x_next + Ti * F([x_next;u]);
        tmp = (eye(8) + Ti * Fs([x_next;u])) * [A, B] + [zeros(8), Ti * Fv([x_next;u])];
        A = tmp(:,1:8);
        B = tmp(:,9:10);
    end
end


