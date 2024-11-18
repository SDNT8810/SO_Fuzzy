
%% Time and Counter Parameters
T_s = 0.01;                     % Time step
T_f = 0.1;                       % Final Time
T_b = 0;                        % Break Time
T_k = 2;                        % Window Time
max_expected_size = T_f / T_s;
Run_Timer = zeros(max_expected_size,1);   % Time vector
Step_Counter = 0;
Window_Size = T_k / T_s;

%% Initial State
x_0 = 0;
x_dot_0 = 0;
y_0 = 0;
y_dot_0 = 0;
theta_0 = 0;
theta_dot_0 = 0;
X0 = [x_0, x_dot_0, y_0, y_dot_0, theta_0, theta_dot_0]';

%% Init State Recorder Matrixes
X = zeros(length(X0), max_expected_size);
X(:,1) = X0;
X_g = zeros(3, max_expected_size);
X_g = X_g + [10;10;0];
Xd0 = [X_g(1), x_dot_0, X_g(2), y_dot_0, X_g(3), theta_dot_0]';
Xd = zeros(length(X0), Window_Size);
Xd(:,1) = Xd0 ;
Dist2Goal = zeros(1, max_expected_size);
Dist2Goal(1) = dist2goal([X(1,1), X(3,1)],X_g);
Xd_sim = zeros(length(X0), Window_Size); 
Xd_sim = Xd_sim + Xd0;

%% Robot Parameters
Lidar_Range = 5;
m = 2;
Robot.m = m;
Robot.Lidar_Range = Lidar_Range;
Robot.X = X0;
Robot.Xd = X0;
Robot.Lidar_Range = -180:1:180;


Robot_Sim = Robot;
Robot_Sim.Window_Size = Window_Size;
Robot_Sim.Xd_sim = Xd_sim;

%% Fuzzy Network Parameters
Number_of_Membership_Functions = 7;
NMF = Number_of_Membership_Functions;
Number_of_Rulls = 0;
Max_Number_of_Rulls = 100;
Number_of_Inputs = 6;    % ........
Number_of_Outputs = 2;    % Xd & Yd
W = zeros(Number_of_Rulls,Number_of_Outputs);
FN_Phi = zeros(Number_of_Rulls,1);
Membership_Functions_Params = zeros(Number_of_Inputs, 2 * Number_of_Membership_Functions);;
Gamma = 0.8;
sample = Robot.X;

Dist_MF_L2F = 30;

Num_MF_L2F = 360/Dist_MF_L2F+1;
Membership_Lidar = zeros(2*Dist_MF_L2F+1 , Num_MF_L2F);
MF_Lidar_ = zeros(Num_MF_L2F, 1);
MF_L2F(:,1) = gaussmf(-Dist_MF_L2F:Dist_MF_L2F , [(Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Lidar_Augmented = @(x) [x(end-Dist_MF_L2F:end,1); x; x(1:Dist_MF_L2F,1)];
Max_Lidar = sum(Lidar_Range * MF_L2F);
MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar);
function o = Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar)
  Y = Lidar_Augmented(Points360);
  for j = 1:Num_MF_L2F
    y = Y((j-1)*Dist_MF_L2F+1: (j+1)*Dist_MF_L2F+1);
    Membership_Lidar(:,j) = MF_L2F.*y;
    MF_Lidar_(j,1) = sum(Membership_Lidar(:,j));
  end
  o = MF_Lidar_/Max_Lidar;
end


%% Environmental Parameters
X_Obstacles_0 = [1, 2
                 2, 5
                 3, 6
                 4, 2
                 5, 1
                 5, 8];
X_Obstacles = zeros(size(X_Obstacles_0,1), 2, max_expected_size);
X_Obstacles = X_Obstacles + X_Obstacles_0;
X_Near_Obstacles = Obstacle_Is_Near([X(1,1),X(3,1)], X_Obstacles(:,:,1), Lidar_Range);
Points360 = zeros(360,1);
%%

