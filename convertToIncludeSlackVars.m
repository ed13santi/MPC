function long = convertToIncludeSlackVars(short, nSlackVars)
    short = [0;0;short];  % 00xuxuxuxuxux
    resized = reshape(short,10,[]); % 00x ux ux ux ux
    resized = [resized; zeros(nSlackVars, size(resized,2))]; % 00xX uxX uxX uxX
    long = reshape(resized, size(resized,1)*size(resized,2), 1); % 00xXuxXuxXuxX
    long = long(3:end); % xXuxXuxXuxX
end


