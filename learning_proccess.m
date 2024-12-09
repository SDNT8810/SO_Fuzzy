
%% simulation and cost calculation
fis.W = W;
fis.M = MeanMat;
fis.V = VariMat;
fis.FuzzySysInputs = FuzzySysInputs;
fis.MF = MF;
fis.T_s = T_s;
fis.ElavFuz = ElavFuz;
fis.Omega = Omega;

env.m2p = m2p;
env.map_local = map_local;
env.Lidar_Range = Lidar_Range;
env.Lidar_Range_near = Lidar_Range_near;
env.MF_Lidar = MF_Lidar;
env.MF_Lidar_Angle = MF_Lidar_Angle;

X_MFW.X = X(:,Step_Counter);
X_MFW.X_g_sim = X_g(:,Step_Counter:Step_Counter+Window_Size-1);

[X_Sim, Cost] = simulate_MFW(X_MFW, env, fis, Window_Size);

% Cost

%% 

        


