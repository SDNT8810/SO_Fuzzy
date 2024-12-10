
function len = dist2goal(X,X_g)

    x = X(1);
    y = X(2);
    xg = X_g(1);
    yg = X_g(2);
    
    len = sqrt(((x-xg)^2)+((y-yg)^2));

end
