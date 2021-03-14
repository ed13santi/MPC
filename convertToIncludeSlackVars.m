function long = convertToIncludeSlackVars(short, nSlackVars)
    short = [0;0;short];
    resized = reshape(short,10,[]);
    resized = [resized; zeros(nSlackVars, size(resized,2))];
    long = reshape(resized, size(resized,1)*size(resized,2), 1);
    long = long(3:end);
end
