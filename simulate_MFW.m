function [states, Cost] = simulate_MFW(State, Action, Params)
    % Parameters
    Window_Size = Params.Window_Size;
    FuzzySysInputs = State.FuzzySysInputs;
    X = State.X;
    X_g = State.X_g;
    W = Action.W;
    T_s = Params.T_s;

    NX = zeros(size(X,1),Window_Size);
    Goal_Vector = zeros(2,Window_Size);
    r = zeros(1,Window_Size);
    RulesNum = length(W);
    Antcs = zeros(RulesNum,1);

    for N = 1 : Window_Size
        Step_Counter = N;
        
        for i = 1:RulesNum
            Antcs(i,1) = Antc(FuzzySysInputs,Action.M(:,i),Action.V(:,i),Params.MF);
        end
        if sum(Antcs) == 0
            Fuzzy_Local_Direction_ref = 0;
        else
            Fuzzy_Local_Direction_ref = Params.ElavFuz(W,Antcs);
        end
        Vel = sqrt(X(2)^2 + X(4)^2);
        NX(2,N) = Vel * cosd(X(5));
        NX(4,N) = Vel * sind(X(5));
        NX(1,N) = X(1) + NX(2,N) * T_s;
        NX(3,N) = X(3) + NX(4,N) * T_s;
        NX(6,N) = (Fuzzy_Local_Direction_ref) * Params.Omega;
        while ((NX(6,N)<-180) || (NX(6,N)>180))
            NX(6,N)  = (NX(6,N)  + (360*(NX(6,N) <-180)) + (-360*(NX(6,N) >180)));
        end
        NX(5,N) = X(5) + NX(6,N) * T_s;
        while ((NX(5,N)<-180) || (NX(5,N)>180))
            NX(5,N)  = (NX(5,N)  + (360*(NX(5,N) <-180)) + (-360*(NX(5,N) >180)));
        end
        X = NX(:,N);
        Points360 = Read_Lidar(X, Params.m2p, Params.Lidar_Range, Params.map_local);
        Points360 = Params.Lidar_Range - Points360;

        Goal_Vector(:,1) = [X_g(1,1) - X(1);X_g(2,1) - X(3)];
        Goal_angle = atan2d(Goal_Vector(2,Step_Counter),Goal_Vector(1,Step_Counter));
        Goal_Direction = Goal_angle - X(5);
        Goal_Direction = (Goal_Direction + (360*(Goal_Direction<-180)) + (-360*(Goal_Direction>180)));

        MF_Lid = Params.MF_Lidar(Points360);

        Phi2Goal = Params.MF_Lidar_Angle - Goal_Direction;
        SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));
        weight_MF = gbellmf(SaturatedPHI2Goal,[70, 3, 0]);
        
        Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);
        Preference_MF = Preference_MF / max(Preference_MF);

        % FuzzySysInputs = [Preference_MF;Goal_Direction/180;dist2goal(X,X_g(:,Step_Counter));X(5)/180];
        FuzzySysInputs = [Preference_MF;Goal_Direction/180;X(5)/180];

        r(1,Step_Counter) = norm(Goal_Vector(:,Step_Counter)) + ...
                            abs(Goal_Direction)/180 + ...
                            (Fuzzy_Local_Direction_ref^2)/180 + ...
                            (0.01/max(min(Params.Lidar_Range - Points360),0.01)^2);
    end
    states.FuzzySysInputs = FuzzySysInputs;
    states.X = reshape(NX,[size(X,1),Window_Size]);
    states.X_g = X_g;
    Cost = r * Params.lambda;
end


