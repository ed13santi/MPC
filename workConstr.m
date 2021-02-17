function [c,ceq] = workConstr(w, u_len, N)
    w = w(1:end-8);
    c = [];
    for i=0:u_len+8:(u_len+8)*(N-1)
        c = [ c;
              w(i+2) * w(i+9) - w(i+11);
              w(i+4) * w(i+10) - w(i+12); ];
    end
    ceq = [];
end