function out = objFuncPath(w, w0, N)
    w_r = [];
    w0_r = [];
    for i=1:N+1
        w_r = [w_r; w(10*i-9); w(10*i-7)];
        w0_r = [w0_r; w0(10*i-9); w0(10*i-7)];
    end
    diff = w_r - w0_r;
    out = diff' * diff;
end


%% manipulate vectors

