function [q, done] = Q(State, Action, Params)

    Q_MF_Lid = Params.Q_MF_Lidar(State.Points360);
    Q_Phi2Goal = Params.Q_MF_Lidar_Angle - State.Goal_Direction;
    Q_Phi2Action = Params.Q_MF_Lidar_Angle - Action.Fuzzy_Local_Direction_ref;
    
    Q_SaturatedPHI2Goal = (Q_Phi2Goal + (360*(Q_Phi2Goal<-180)) + (-360*(Q_Phi2Goal>180)));
    Q_SaturatedPHI2Action = (Q_Phi2Action + (360*(Q_Phi2Action<-180)) + (-360*(Q_Phi2Action>180)));
    
    Q_weight_MF = gbellmf(Q_SaturatedPHI2Goal,[70, 3, 0]);
    Q_weight_MF_A = gbellmf(Q_SaturatedPHI2Action,[70, 3, 0]);
    
    Q_Preference_MF = (Q_weight_MF*0.7+0.3).*(1-Q_MF_Lid).*(Q_weight_MF_A*0.7+0.3);
    Q_Preference_MF = Q_Preference_MF / max(Q_Preference_MF);
    
    q = -10;
    % -10*((5*max(Q_Preference_MF)-sum(Q_Preference_MF)) + ...
    %     dist2goal([State.X(1), State.X(3)], State.X_g) + ...
    %     1/min(Params.Lidar_Range - State.Points360) + ...
    %     (State.Goal_Direction/90)^2);

    done = true;

    if (dist2goal([State.X(1), State.X(3)], State.X_g) < Params.R)
        done = false;
        q = 100;
    end
    if (min(Params.Lidar_Range - State.Points360) < 3*Params.R) 
        done = false;
        q = -100;
    end
    
end