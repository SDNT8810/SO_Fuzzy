x = X(1);
    y = X(3);
    t = X(5);
    l = Lidar_Range;
    s = size(map);
    m2p = s(1) / X_g(1);
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


% plot(Antcs)
% x = 0:0.001:1;
% for j = 1 : RulesNum
%     clf
%     for i = 1 : Num_MF_L2F
%         pp(i,:) = gaussmf(x , [VariMat(i,j), MeanMat(i,j)]);
%     end
%
%     for i = 1 : Num_MF_L2F
%         hold on
%         plot(x,pp(i,:))
%     end
%     pause(0.4)
% end
%
%
% for j = 1 : Num_MF_L2F
%     clf
%     for i = 1 : RulesNum
%         pp(i,:) = gaussmf(x , [VariMat(j,i), MeanMat(j,i)]);
%     end
%
%     for i = 1 : RulesNum
%         hold on
%         plot(pp(i,:))
%     end
%     pause(0.1)
% end

