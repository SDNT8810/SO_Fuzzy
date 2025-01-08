%% init
clc
figure(2)
clf

%% Parameters
subplot(221)

Goal_angle = 90;

Dist_MF_L2F = 30;
Num_MF_L2F = 360/Dist_MF_L2F;
Membership_Lidar = zeros(2*Dist_MF_L2F+1 , Num_MF_L2F);
MF_Lidar_ = zeros(Num_MF_L2F, 1);
MF_L2F(:,1) = gaussmf(-Dist_MF_L2F:Dist_MF_L2F , [(Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Lidar_Augmented = @(x) [x(end-Dist_MF_L2F:end,1); x; x(1:Dist_MF_L2F,1)];
Max_Lidar = sum(Lidar_Range * MF_L2F);
MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar);

num_gaussians = 13; % Number of Gaussian filters
Dist_MF_L2F = 30; % Distance between membership functions (angles)
theta = linspace(0, 2*pi, 360); % 360 LiDAR angles
MF_Lidar_Angle = (0:Dist_MF_L2F:359)'; % Centers of Gaussian filters in degrees
MF_Lid = MF_Lidar(Points360Plot);
MF_Lid(13) = MF_Lid(1);
MF_Lidar_Angle(13) = 360;

%% Plot Weighting
gaussian_width = Dist_MF_L2F * 0.8; % Gaussian width
i = 1;
center_angle = Goal_angle; % Center of the Gaussian filter
shifted_theta = mod(theta - deg2rad(center_angle) + pi, 2*pi) - pi; % Periodic adjustment
gaussian_shape = 0.3 + 0.7 * exp(-shifted_theta.^2 / (6 * (deg2rad(gaussian_width))^2));

% Convert Gaussian shape to Cartesian
x_gaussian = gaussian_shape .* cos(theta);
y_gaussian = gaussian_shape .* sin(theta);
Weighting = plot(x_gaussian, y_gaussian, 'blue--', 'LineWidth', 2.5); % Gaussian plot
R = 1.5;
hold on;






