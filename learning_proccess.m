
%% Define RL System
% Actor parameters
Action.W = W;
Action.M = MeanMat;
Action.V = VariMat;
Action.Fuzzy_Local_Direction_ref = Fuzzy_Local_Direction_ref;

% States
State.FuzzySysInputs = FuzzySysInputs;
State.Points360 = Points360;
State.X = X(:,Step_Counter);
State.X_g = X_g(:,Step_Counter);
% Params.map_local = map_local;

%% simulation and cost calculation
[State_sim, Cost] = simulate_MFW(State, Action, Params);
        
% [State_Prim, Action_Prim] = simulate_OSA(State, Action, Params);


