function Lidar_Points = Read_Lidar(X, m2p, Lidar_Range, map)
    x = X(1);
    y = X(3);
    t = X(5);
    dl = floor(Lidar_Range * m2p);
    s = size(map);
    px = x * m2p + 1;
    py = y * m2p + 1;
    Points360 = Lidar_Range * ones(360,1);
    for i = 1 : 360
        nt = t + i;
        for j =  1 : dl
            nx = round(px + j * cosd(nt));
            ny = round(py + j * sind(nt));
            if ((nx < 1) || (ny < 1) || (nx > s(1)) || (ny > s(2)))
                Points360(i) = (j/dl) * Lidar_Range;
                break
            end
            if (map(nx,ny)==0) 
                Points360(i) = (j/dl) * Lidar_Range;
                break
            end
        end
    end
    Lidar_Points = Points360;
end


