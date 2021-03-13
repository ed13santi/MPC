function p = changeToCorrectLength(path, N)
    N_path = size(path,2) - 1;
    intStep = N_path / N;
    p(1,:) = interp1(1:N_path+1, path(1,:), 1:intStep:N_path+1,'linear');
    p(2,:) = interp1(1:N_path+1, path(2,:), 1:intStep:N_path+1,'linear');
end

