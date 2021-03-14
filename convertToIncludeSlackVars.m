function long = convertToIncludeSlackVars(short, nSlackVars);
    short = [0;0;short];
    resized = reshape(short,10,[]);
    resized = [resized; zeros(nSlackVars, size(resized,2))];
    long = reshape(resized, [], 1);
    long = long(3:end);
end
