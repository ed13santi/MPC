function w_out = interpolate(w, factor) %how does interp work
    w = [w; 0; 0];
    intStep = 1/factor;
    N = length(w)/10;
    w = reshape(w,[10,N]);
    w_out = [];
    for i=1:10
        interpolatedVec = interp1(1:N, w(i,:), 1:intStep:N,'linear');
        w_out = [ w_out; interpolatedVec ];
    end
    w_out = reshape(w_out,[],1) ;
    w_out = w_out(1:end-2);
end
