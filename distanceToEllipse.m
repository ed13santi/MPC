function dist = distanceToEllipse(x,y,xc,yc,a,b)
    beta = 1 / (((x-xc)/a)^2 + ((y-yc)/b)^2);
    distToCentre = sqrt((x-xc)^2 + (y-yc)^2);
    dist = distToCentre * (1-sqrt(beta)); 
end
