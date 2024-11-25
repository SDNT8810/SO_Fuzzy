function Lidar_Points = Read_Lidar(X, Lidar_Range, map)
    x = X(1);
    y = X(3);
    t = X(5);
    m2p = 100;
    l = Lidar_Range * m2p;
    s = size(map);
    dl = l * m2p;
    rdl = floor(dl+1);
    map_Augmented = ones(2*rdl+s(1)+4,2*rdl+s(2)+4);
    map_Augmented(rdl+1:rdl+s(1),rdl+1:rdl+s(2)) = map .* map_Augmented(rdl+1:rdl+s(1),rdl+1:rdl+s(2));
    px = rdl + x * m2p;
    py = rdl + y * m2p;
    Points360 = zeros(360,1);
    for i = 1 : 360
        nt = t + i;
        for j =  1 : dl
            nx = x + l * (j/dl) * cosd(nt);
            ny = y + l * (j/dl) * sind(nt);
            nxp = max(floor(rdl + nx * m2p),1);
            nyp = max(floor(rdl + ny * m2p),1);
            nxp = min(nxp,size(map_Augmented,1));
            nyp = min(nyp,size(map_Augmented,2));
            dxp = nxp - px;
            dyp = nyp - py;
            if (map_Augmented(nxp,nyp)==0) 
                Points360(i) = (1 - j/dl) * l;
                break
            end
        end
    end
    Lidar_Points = Points360;
end


