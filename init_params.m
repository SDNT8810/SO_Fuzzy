%% General Params
Debug_Mode = 0;
Gazebo_Sim = 0;
EliminateDoyToHighCost  = false;
EliminateDoToAge        = true;
EliminateDoToSimilarity = true;

%% Time and Counter Parameters
T_s = 0.02;                               % Time step
T_f = 100;                                % Final Time
T_b = 0;                                  % Break Time
Window_Size = 10;
Step_Counter = 0;
max_expected_size = round(T_f / T_s);
Run_Timer = zeros(max_expected_size,1);   % Time vector
T_k = Window_Size * T_s;                  % Window Time
t = linspace(0,2*pi,50);
Delta_t = 1;

%% Initial State
V = 0.25;
Omega = 4 * pi;
x_0 = .1;
x_dot_0 = V;
y_0 = .3;
y_dot_0 = V;
theta_0 = 0;
theta_dot_0 = Omega;
X0 = [x_0, x_dot_0, y_0, y_dot_0, theta_0, theta_dot_0]';

%% Init State Recorder Matrixes
X = X0 + zeros(length(X0), max_expected_size);
X_g = [1.5;1.6;0] + zeros(3, max_expected_size);
Xd0 = [X_g(1), x_dot_0, X_g(2), y_dot_0, X_g(3), theta_dot_0]';
Xd = Xd0 + zeros(length(X0), max_expected_size);
Dist2Goal = dist2goal([X(1,1), X(3,1)],X_g) + zeros(1, max_expected_size);
Xd_sim = zeros(length(X0), Window_Size) + Xd0;
Goal_Vector = zeros(2, max_expected_size);
Goal_Vector_sim = zeros(2, Window_Size);

%% Robot Parameters
Lidar_Range = 0.7;
m = 2;
Robot.m = m;
Robot.Lidar_Range = Lidar_Range;
Robot.X = X0;
Robot.Xd = X0;
Robot.Lidar_Range = -179:1:180;
Robot.Heading = theta_0;
Robot.R = .05;

Robot_Sim = Robot;
Robot_Sim.Window_Size = Window_Size;
Robot_Sim.Xd_sim = Xd_sim;

%% Fuzzy Network Parameters
Number_of_Rulls = 0;
Max_Number_of_Rulls = 70;
bell_size = 70;
bell_coff = 3;
sample = Robot.X;

Dist_MF_L2F = 30;
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
Var0 = 1.2;
W = [];
RulesNum = 0;
MF = @(X,M,S) gaussmf(X,[S, M]);
Phi_a = @(Antcs) Antcs/abs(sum(Antcs));
ElavFuz = @(W,Antcs) 180 * W' * Phi_a(Antcs);

temp_w = 0;
gamma = 0.65;
min_gamma = 0.2;
max_age = 250;
min_similarity = 0.6;
Cost = 0;
max_aloable_cost = 250;

gamma_0 = 0.9;
lambda = gamma_0 .^ (1:Window_Size)';
epsilon = 0.01;
gamma_RL = 0.9;
Gamma_RL = diag(gamma_RL.^(1:Window_Size));
alpha = 0.001;

%% Q_Fuzzy Network Parameters
Q_Dist_MF_L2F = 90;
Q_MF_Lidar_Angle = (0:Q_Dist_MF_L2F:359)';

Q_Num_MF_L2F = 360/Q_Dist_MF_L2F;
Q_Membership_Lidar = zeros(2*Q_Dist_MF_L2F+1 , Q_Num_MF_L2F);
Q_MF_Lidar_ = zeros(Q_Num_MF_L2F, 1);
Q_MF_L2F(:,1) = gaussmf(-Q_Dist_MF_L2F:Q_Dist_MF_L2F , [(Q_Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Q_Lidar_Augmented = @(x) [x(end-Q_Dist_MF_L2F:end,1); x; x(1:Q_Dist_MF_L2F,1)];
Q_Max_Lidar = sum(Lidar_Range * Q_MF_L2F);
Q_MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Q_Lidar_Augmented, Q_Membership_Lidar, Q_MF_Lidar_, Q_Dist_MF_L2F, Q_Num_MF_L2F, Q_MF_L2F, Q_Max_Lidar);
Q_RulesNum = (Q_Num_MF_L2F ^ 2);

% Q_FuzzySysInputs = [Q_Preference_MF;Goal_Direction/180;Robot.Heading/180];
Q_MeanMat = zeros(Q_Num_MF_L2F+2, Q_RulesNum);
Q_VariMat = ones(Q_RulesNum,1);
% Q_MeanMat(:,1) = linspace(-1,1,Q_RulesNum);
% Q_MeanMat = [-1 
% 
%                         ];
% Q_VariMat = ones(Q_RulesNum,1);
% 

%% Environmental Parameters
if (Gazebo_Sim == 1)
    m2p = 20;
    Ros_Gazebo

else
    % Lidar_Range = 2;
    % figure(2)
    subplot(2,3,[1,2,4,5])
    clf
    map_rgb = imread('maps/simple.png');
    % map_rgb = imread('maps/Room_map.png');
    % map_rgb = imread('maps/star_300.png');
    subplot(2,3,[1,2,4,5])
    map_bin = imbinarize(map_rgb, 0.95);
    map = map_bin(:,:,1);
    map = transpose(map);
    s = size(map);
    map_local = map;
    map_wide = 3;    
    m2p = round(s(1)/map_wide);
    b = m2p * Robot.R * 0.9;
    bb = b * Lidar_Range;
    c = Lidar_Range*m2p;
    rl = floor(Lidar_Range * m2p);
    % map_frame = ones(2*l + s);
    % map_frame(l+1:l+s(1),l+1:l+s(2)) = map;

    hImage = imshow(map', 'InitialMagnification', 'fit');
    
    % Adjust transparency using 'AlphaData'
    alphaValue = 0.3; % Set transparency level (0 = fully transparent, 1 = fully opaque)
    set(hImage, 'AlphaData', alphaValue);

    hold on;
    startPoint = plot(m2p * X(1,1), m2p * X(3,1), 's', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', 'MarkerSize', 20); % Green marker for start
    targetPoint = plot(m2p * X_g(1,1), m2p * X_g(2,1), 'p', 'MarkerFaceColor', 'r', 'MarkerSize', 40); % Red marker for goal
    lidar_circle = plot(c*sin(t)+(X(1,1)),c*cos(t)+X(3,1),':b','linewidth',0.8);
    delete(lidar_circle);
end

Points360 = zeros(360,1);
x = zeros(1,max_expected_size);
y = zeros(1,max_expected_size);
theta = zeros(1,max_expected_size);

lidar_index = 360;
lidar_line(lidar_index) = line([0; 0.1], [0.1; 0.1], 'color', 'blue');
delete(lidar_line);
lw = 5;

% Get the size of the map
[rows, cols] = size(map);

% Define the rectangle around the map
rectangle('Position', [0.5, 0.5, rows, cols], 'EdgeColor', 'black', 'LineWidth', 5);

% Add title and axis labels
title(['Time: ', num2str(T_s * Step_Counter)], 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('X-axis (cm)', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Y-axis (cm)', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

% Adjust the axes to ensure ruler-like appearance
ax = gca; % Get current axes
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridColor = 'black'; % Grid color
ax.GridAlpha = 1.0; % Grid transparency
ax.LineWidth = 1.5; % Grid line thickness
ax.GridLineStyle = '-'; % Dashed grid lines
ax.GridAlpha = 0.05; % Transparency of grid lines

% Enable axis ticks and labels
axis on;
set(ax, 'TickDir', 'out', 'FontSize', 24, 'FontWeight', 'bold'); % Customize tick appearance

% Add legend
legend([startPoint, targetPoint], {'Start Point', 'Target'}, 'FontSize', 24, 'Location', 'best');

%% RL Parameters
Long_Memory_Experience.Window_Size = Window_Size;
load FuzzySystemLongMemory
Params.Window_Size = Window_Size;
Params.T_s = T_s;
Params.Omega = Omega;
Params.m2p = m2p;
Params.lambda = lambda;
Params.MF_Lidar_Angle = MF_Lidar_Angle;
Params.Q_MF_Lidar_Angle = Q_MF_Lidar_Angle;
Params.Lidar_Range = Lidar_Range;
Params.map_local = map_local;
Params.MF_Lidar = MF_Lidar;
Params.Q_MF_Lidar = Q_MF_Lidar;
Params.MF = MF;
Params.ElavFuz = ElavFuz;
Params.Phi_a = Phi_a;
Params.epsilon = epsilon;
Params.Delta_t = Delta_t;
Params.gamma_RL = gamma_RL;
Params.Gamma_RL = Gamma_RL;
Params.alpha = alpha;
Params.R = Robot.R;
Params.gamma_0 = gamma_0;

