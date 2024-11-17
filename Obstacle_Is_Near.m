
function Near_Obstacles = Obstacle_Is_Near(Robot_Position, X_Obstacles, Lidar_Range)
    
    I_max = length(X_Obstacles);
    dist = zeros(I_max,1);
    for i = 1 : I_max
        dist(i) = dist2goal(Robot_Position,X_Obstacles(i,:));
    end    
    Near_Obstacles = X_Obstacles((dist<Lidar_Range),:);
end
