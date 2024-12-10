clc


N = 1;
fis.W = W;
fis.M = MeanMat;
fis.V = VariMat;
fis.FuzzySysInputs = FuzzySysInputs;
fis.MF = MF;
fis.T_s = T_s;
Window_Size = 5;
XX = X(:,5);
env = [0,0];


[states, New_env] = simulate_MFW(XX, env, fis, N, Window_Size);

[states XX]




function [states, New_env] = simulate_MFW(X, env, fis, N, Window_Size, ElavFuz)
    % 
    % x = X(1);
    % y = X(3);
    % phi = X(5); 
    % 
    % local_map = env.local_map;
    % global_map = env.global_map;
    % Lidar_data = env.Lidar_data;
    % 
    % W = fis.W;
    % M = fis.M;
    % V = fis.V;
    % FuzzySysInputs = fis.FuzzySysInputs;
    % MF = fis.MF;
    T_s = fis.T_s;
    % 
    
    if N < Window_Size
        % N
        % onestepahead;
        RulesNum = length(fis.W);
        Antcs = zeros(RulesNum,1);
        for i = 1:RulesNum
            Antcs(i) = Antc(fis.FuzzySysInputs,fis.M(:,i),fis.V(:,i),fis.MF);
        end
        if sum(Antcs) == 0
            Fuzzy_Local_Direction_ref = 0;
        else
            Fuzzy_Local_Direction_ref = ElavFuz(fis.W,Antcs);
        end
        Vel = sqrt(X(2)^2 + X(4)^2);
        NX(2) = Vel * cosd(X(5));
        NX(4) = Vel * sind(X(5));
        NX(1) = X(1) + NX(2) * T_s;
        NX(3) = X(3) + NX(4) * T_s;
        NX(6) = (Fuzzy_Local_Direction_ref) * X(6) * pi / 180;
        NX(5) = X(5) + NX(6) * T_s;
        NX(5)  = (NX(5)  + (360*(NX(5) <-180)) + (-360*(NX(5) >180)));
        [NX, Nenv] = simulate_MFW(NX, env, fis, N+1, Window_Size, ElavFuz);
        env = [0,0];
        states = reshape(NX,[6,1]);
        New_env = Nenv;
    else
        states = reshape(X,[6,1]);
        New_env = env;
    end

end




% x = X(1);
%     y = X(3);
%     t = X(5);
%     l = Lidar_Range;
%     s = size(map);
%     m2p = s(1) / X_g(1);
%     dl = l * m2p;
%     rdl = floor(dl+1);
%     map_Augmented = ones(2*rdl+s(1)+4,2*rdl+s(2)+4);
%     map_Augmented(rdl+1:rdl+s(1),rdl+1:rdl+s(2)) = map .* map_Augmented(rdl+1:rdl+s(1),rdl+1:rdl+s(2));
%     px = rdl + x * m2p;
%     py = rdl + y * m2p;
%     Points360 = zeros(360,1);
%     for i = 1 : 360
%         nt = t + i;
%         for j =  1 : dl
%             nx = x + l * (j/dl) * cosd(nt);
%             ny = y + l * (j/dl) * sind(nt);
%             nxp = max(floor(rdl + nx * m2p),1);
%             nyp = max(floor(rdl + ny * m2p),1);
%             nxp = min(nxp,size(map_Augmented,1));
%             nyp = min(nyp,size(map_Augmented,2));
%             dxp = nxp - px;
%             dyp = nyp - py;
%             if (map_Augmented(nxp,nyp)==0) 
%                 Points360(i) = (1 - j/dl) * l;
%                 break
%             end
%         end
%     end
%     Lidar_Points = Points360;
% 
% 
% % plot(Antcs)
% % x = 0:0.001:1;
% % for j = 1 : RulesNum
% %     clf
% %     for i = 1 : Num_MF_L2F
% %         pp(i,:) = gaussmf(x , [VariMat(i,j), MeanMat(i,j)]);
% %     end
% %
% %     for i = 1 : Num_MF_L2F
% %         hold on
% %         plot(x,pp(i,:))
% %     end
% %     pause(0.4)
% % end
% %
% %
% % for j = 1 : Num_MF_L2F
% %     clf
% %     for i = 1 : RulesNum
% %         pp(i,:) = gaussmf(x , [VariMat(j,i), MeanMat(j,i)]);
% %     end
% %
% %     for i = 1 : RulesNum
% %         hold on
% %         plot(pp(i,:))
% %     end
% %     pause(0.1)
% % end
% 
