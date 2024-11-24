
%% Time and Counter Parameters
T_s = 0.05;                     % Time step
T_f = 10;                       % Final Time
T_b = 0;                        % Break Time
T_k = 0.1;                        % Window Time
max_expected_size = round(T_f / T_s);
Run_Timer = zeros(max_expected_size,1);   % Time vector
Step_Counter = 0;
Window_Size = round(T_k / T_s);

%% Initial State
x_0 = 0;
x_dot_0 = 2;
y_0 = 0;
y_dot_0 = 2;
theta_0 = 70;
theta_dot_0 = 0;
X0 = [x_0, x_dot_0, y_0, y_dot_0, theta_0, theta_dot_0]';

V = sqrt(x_dot_0^2 + y_dot_0^2);
Omega = theta_dot_0;

%% Init State Recorder Matrixes
X = X0 + zeros(length(X0), max_expected_size);
X_g = [300;300;0] + zeros(3, max_expected_size);
Xd0 = [X_g(1), x_dot_0, X_g(2), y_dot_0, X_g(3), theta_dot_0]';
Xd = Xd0 + zeros(length(X0), max_expected_size);
Dist2Goal = dist2goal([X(1,1), X(3,1)],X_g) + zeros(1, max_expected_size);
Xd_sim = zeros(length(X0), Window_Size) + Xd0;
Goal_Vector = zeros(2, max_expected_size);
Goal_Vector_sim = zeros(2, Window_Size);

%% Robot Parameters
Lidar_Range = 30;
m = 2;
Robot.m = m;
Robot.Lidar_Range = Lidar_Range;
Robot.X = X0;
Robot.Xd = X0;
Robot.Lidar_Range = -180:1:180;
Robot.Heading = theta_0;

Robot_Sim = Robot;
Robot_Sim.Window_Size = Window_Size;
Robot_Sim.Xd_sim = Xd_sim;

%% Fuzzy Network Parameters
Number_of_Membership_Functions = 7;
NMF = Number_of_Membership_Functions;
Number_of_Rulls = 0;
Max_Number_of_Rulls = 100;
Number_of_Inputs = 6;    % ........
Number_of_Outputs = 2;    
W = zeros(Number_of_Rulls,Number_of_Outputs);
FN_Phi = zeros(Number_of_Rulls,1);
Membership_Functions_Params = zeros(Number_of_Inputs, 2 * Number_of_Membership_Functions);
Gamma = 0.8;
bell_size = 70;
bell_coff = 3;
sample = Robot.X;

Dist_MF_L2F = 15;
MF_Lidar_Angle = (0:Dist_MF_L2F:359)';

Num_MF_L2F = 360/Dist_MF_L2F;
Membership_Lidar = zeros(2*Dist_MF_L2F+1 , Num_MF_L2F);
MF_Lidar_ = zeros(Num_MF_L2F, 1);
MF_L2F(:,1) = gaussmf(-Dist_MF_L2F:Dist_MF_L2F , [(Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Lidar_Augmented = @(x) [x(end-Dist_MF_L2F:end,1); x; x(1:Dist_MF_L2F,1)];
Max_Lidar = sum(Lidar_Range * MF_L2F);
MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar);

MeanMat = [];
VariMat = [];
Var0 = 1;
W = [];
RulesNum = 0;
MF = @(X,M,S) gaussmf(X,[S, M]);


%% Environmental Parameters
map_rgb = imread('maps/star_300.png');
subplot(2,2,[1,3])
imshow(map_rgb);
map_bin = imbinarize(map_rgb, 0.95);
map = map_bin(:,:,1);

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

