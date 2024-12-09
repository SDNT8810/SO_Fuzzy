function [states, Cost] = simulate_MFW(X_MFW, env, fis, Window_Size)
    X = X_MFW.X;
    X_g_sim = X_MFW.X_g_sim;
    
    FuzzySysInputs = fis.FuzzySysInputs;
    W = fis.W;
    NX = zeros(size(X,1),Window_Size);
    Goal_Vector = zeros(2,Window_Size);
    J = zeros(1,Window_Size);
    RulesNum = length(W);
    Antcs = zeros(RulesNum,1);

    for N = 1 : Window_Size
        Step_Counter = N;
        
        for i = 1:RulesNum
            Antcs(i,1) = Antc(FuzzySysInputs,fis.M(:,i),fis.V(:,i),fis.MF);
        end
        if sum(Antcs) == 0
            Fuzzy_Local_Direction_ref = 0;
        else
            Fuzzy_Local_Direction_ref = fis.ElavFuz(W,Antcs);
        end
        Vel = sqrt(X(2)^2 + X(4)^2);
        NX(2,N) = Vel * cosd(X(5));
        NX(4,N) = Vel * sind(X(5));
        NX(1,N) = X(1) + NX(2,N) * fis.T_s;
        NX(3,N) = X(3) + NX(4,N) * fis.T_s;
        NX(6,N) = (Fuzzy_Local_Direction_ref) * fis.Omega;
        while ((NX(6,N)<-180) || (NX(6,N)>180))
            NX(6,N)  = (NX(6,N)  + (360*(NX(6,N) <-180)) + (-360*(NX(6,N) >180)));
        end
        NX(5,N) = X(5) + NX(6,N) * fis.T_s;
        while ((NX(5,N)<-180) || (NX(5,N)>180))
            NX(5,N)  = (NX(5,N)  + (360*(NX(5,N) <-180)) + (-360*(NX(5,N) >180)));
        end
        X = NX(:,N);
        Points360 = Read_Lidar(X, env.m2p, env.Lidar_Range, env.map_local);
        Points360_near = Read_Lidar(X, env.m2p, env.Lidar_Range_near, env.map_local);
        Points360 = env.Lidar_Range - Points360;
        Points360_near = env.Lidar_Range_near - Points360_near;

        Goal_Vector(:,Step_Counter) = [X_g_sim(1,Step_Counter) - X(1);X_g_sim(2,Step_Counter) - X(3)];
        Goal_angle = atan2d(Goal_Vector(2,Step_Counter),Goal_Vector(1,Step_Counter));
        Goal_Direction = Goal_angle - X(5);
        Goal_Direction = (Goal_Direction + (360*(Goal_Direction<-180)) + (-360*(Goal_Direction>180)));

        MF_Lid = env.MF_Lidar(Points360);
        MF_Lid_near = env.MF_Lidar(Points360_near);

        Phi2Goal = env.MF_Lidar_Angle - Goal_Direction;
        SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));
        weight_MF = gbellmf(SaturatedPHI2Goal,[70, 3, 0]);
        
        Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);
        Preference_MF_near = (weight_MF*0.7+0.3).*(1-MF_Lid_near);
        Preference_MF = Preference_MF / max(Preference_MF);
        Preference_MF_near = Preference_MF_near / max(Preference_MF_near);

        FuzzySysInputs = [Preference_MF_near;Preference_MF;Goal_Direction/180;dist2goal(X,X_g_sim(:,Step_Counter));X(5)/180];
        
        NJ = norm(Goal_Vector(:,Step_Counter)) + abs(Goal_Direction)/180 + (Fuzzy_Local_Direction_ref^2)/180 + (0.01/max(min(env.Lidar_Range_near - Points360_near),0.01)^2);
        J(1,Step_Counter) = NJ;
    end
    states.X = reshape(NX,[size(X,1),Window_Size]);
    states.X_g_sim = reshape(X_g_sim,[size(X_g_sim,1),Window_Size]);
    Cost = J * fis.lambda;
end