function [State_Prim, Action_Prim, r] = simulate_OSA(State, Action, Params)
    % Parameters
    FuzzySysInputs = State.FuzzySysInputs;
    % Points360 = State.Points360;
    X = State.X;
    X_g = State.X_g;
    W = Action.W;
    T_s = Params.T_s;

    NX = zeros(size(X,1),1);
    Goal_Vector = zeros(2,1);
    r = zeros(1,1);
    RulesNum = length(W);
    Antcs = zeros(RulesNum,1);

    Vel = sqrt(X(2)^2 + X(4)^2);
    NX(2,1) = Vel * cosd(X(5));
    NX(4,1) = Vel * sind(X(5));
    NX(1,1) = X(1) + NX(2,1) * T_s;
    NX(3,1) = X(3) + NX(4,1) * T_s;
    NX(6,1) = (Action.Fuzzy_Local_Direction_ref) * Params.Omega;
    while ((NX(6,1)<-180) || (NX(6,1)>180))
        NX(6,1)  = (NX(6,1)  + (360*(NX(6,1) <-180)) + (-360*(NX(6,1) >180)));
    end
    NX(5,1) = X(5) + NX(6,1) * T_s;
    while ((NX(5,1)<-180) || (NX(5,1)>180))
        NX(5,1)  = (NX(5,1)  + (360*(NX(5,1) <-180)) + (-360*(NX(5,1) >180)));
    end
    X = NX(:,1);
    
    Goal_Vector(:,1) = [X_g(1,1) - X(1);X_g(2,1) - X(3)];
    Goal_angle = atan2d(Goal_Vector(2,1),Goal_Vector(1,1));
    Goal_Direction = Goal_angle - X(5);
    Goal_Direction = (Goal_Direction + (360*(Goal_Direction<-180)) + (-360*(Goal_Direction>180)));
    
    for i = 1:RulesNum
        Antcs(i,1) = Antc(FuzzySysInputs,Action.M(:,i),Action.V(:,i),Params.MF);
    end
    if rand < Params.epsilon
        Fuzzy_Local_Direction_ref = 360 * (rand-0.5);
    else
        if sum(Antcs) == 0
            Fuzzy_Local_Direction_ref = 0;
        else
            Fuzzy_Local_Direction_ref = Params.ElavFuz(W,Antcs);
        end
    end
    Points360 = Read_Lidar(X, Params.m2p, Params.Lidar_Range, Params.map_local);
    Points360 = Params.Lidar_Range - Points360;

    MF_Lid = Params.MF_Lidar(Points360);

    Phi2Goal = Params.MF_Lidar_Angle - Goal_Direction;
    SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));
    weight_MF = gbellmf(SaturatedPHI2Goal,[70, 3, 0]);
    
    Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);
    Preference_MF = Preference_MF / max(Preference_MF);

    % FuzzySysInputs = [Preference_MF;Goal_Direction/180;dist2goal(X,X_g(:,Step_Counter));X(5)/180];
    FuzzySysInputs = [Preference_MF;Goal_Direction/180;X(5)/180];
    
    r(1,1) = 1/norm(Goal_Vector(:,1)) + ...
             180/abs(Goal_Direction) + ...
             180/(Fuzzy_Local_Direction_ref^2) + ...
             abs(max(min(Params.Lidar_Range - Points360),Params.R)^2);

    State_Prim.FuzzySysInputs = FuzzySysInputs;
    State_Prim.Points360 = Points360;
    State_Prim.X = NX;
    State_Prim.X_g = X_g;
    State_Prim.Phi_a = Params.Phi_a(Antcs);
    State_Prim.Goal_Direction = Goal_Direction;

    Action_Prim.W = W;
    Action_Prim.M = Action.M;
    Action_Prim.V = Action.V;
    Action_Prim.Fuzzy_Local_Direction_ref = Fuzzy_Local_Direction_ref;

end


