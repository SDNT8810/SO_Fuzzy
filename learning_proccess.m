
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
State.Phi_a = Phi_a(Antcs);

% Params.map_local = map_local;

% Critic Network
W_c = 1;
Phi_c = @(S,A) 1;
Q = @(S,A) W_c * Phi_c(S, A);


%% simulation and cost calculation
N = 0;
r = zeros(1,Window_Size);
TD = zeros(1,Window_Size);
DT_DW = zeros(Window_Size,length(W));
while (N < Window_Size)
    N = N + 1;
    [State_Prim, Action_Prim, r(N)] = simulate_OSA(State, Action, Params);
    Experience{N} = {State, Action, r(N), State_Prim, Action_Prim, TD(N), W};
    DT_DW(N,:) = State_Prim.Phi_a';
    % End Conditions
    if (dist2goal([State_Prim.X(1), State_Prim.X(3)], State_Prim.X_g) < 5/Params.m2p)
        TD(N) = 1000;
        break
    end
    if (min(Params.Lidar_Range - Points360) < 0.02) 
        TD(N) = -100;
        break
    end
    
    TD(N) = -Q(State,Action) - Params.Delta_t*r(N) + Params.gamma_RL * Q(State_Prim,Action_Prim);
    State = State_Prim;
    Action = Action_Prim;
end
   
%% Train Actor
T = Choose_Experience(TD,Long_Memory_Experience);

W_new = W - Params.alpha * (T * Params.Gamma_RL * DT_DW)';

%% Evaluate Training
[State_Prim, Action_Prim, r(N)] = simulate_OSA(State, Action, Params);















