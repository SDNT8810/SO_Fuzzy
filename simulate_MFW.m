function [states, New_env] = simulate_MFW(X, env, fis, N, Window_Size)
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
        N
        % onestepahead;
        RulesNum = length(fis.W);
        Antcs = zeros(RulesNum,1);
        for i = 1:RulesNum
            Antcs(i) = Antc(fis.FuzzySysInputs,fis.M(:,i),fis.V(:,i),fis.MF);
        end
        if sum(Antcs) == 0
            Fuzzy_Local_Direction_ref = 0;
        else
            Fuzzy_Local_Direction_ref = (fis.W'*Antcs)/abs(sum(Antcs));
        end
        Vel = sqrt(X(2)^2 + X(4)^2);
        NX(2) = Vel * cosd(X(5));
        NX(4) = Vel * sind(X(5));
        NX(1) = X(1) + NX(2) * T_s;
        NX(3) = X(3) + NX(4) * T_s;
        NX(6) = (Fuzzy_Local_Direction_ref) * X(6) * pi / 180;
        NX(5) = X(5) + NX(6) * T_s;
        NX(5)  = (NX(5)  + (360*(NX(5) <-180)) + (-360*(NX(5) >180)));
        [NX, Nenv] = simulate_MFW(NX, env, fis, N+1, Window_Size);
        env = [0,0];
        states = reshape(NX,[6,1]);
        New_env = Nenv;
    else
        states = reshape(X,[6,1]);
        New_env = env;
    end

end