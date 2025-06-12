%% init
clc
clf
figure(2)

% Add functions directory to path
addpath('../functions');

Dist_MF_L2F = 30;
MF_Lidar_Angle = (0:Dist_MF_L2F:359)';

Num_MF_L2F = 360/Dist_MF_L2F;
Membership_Lidar = zeros(2*Dist_MF_L2F+1 , Num_MF_L2F);
MF_Lidar_ = zeros(Num_MF_L2F, 1);
MF_L2F(:,1) = gaussmf(-Dist_MF_L2F:Dist_MF_L2F , [(Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Lidar_Augmented = @(x) [x(end-Dist_MF_L2F:end,1); x; x(1:Dist_MF_L2F,1)];
Max_Lidar = sum(Lidar_Range * MF_L2F);
MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar);

% Parameters
r_circle = 1; % Radius of the circle
num_gaussians = Num_MF_L2F; % Number of Gaussian filters
Dist_MF_L2F = 30; % Distance between membership functions (angles)
theta = linspace(0, 2*pi, 360); % 360 LiDAR angles
MF_Lidar_Angle = (0:Dist_MF_L2F:359)'; % Centers of Gaussian filters in degrees
MF_Lid = MF_Lidar(Points360Plot);
MF_Lid(13) = MF_Lid(1);
MF_Lidar_Angle(13) = 360;

%% Circle plot
hold on;
theta_circle = linspace(0, 2*pi, 500);
plot(r_circle * cos(theta_circle), r_circle * sin(theta_circle), 'k--', 'LineWidth', 1);

for lidar_index = 1 : 1 : 360
    len = (Lidar_Range-Points360Plot(lidar_index));
    lidar_line(lidar_index) = line([0;len*sind(lidar_index+90)],[0;len*cosd(lidar_index+90)], 'color', 'blue', 'LineWidth', lw/10);
end

% Plot Gaussian filters (Mexican hat)
gaussian_width = Dist_MF_L2F / 4; % Gaussian width
for i = 1:num_gaussians
    center_angle = MF_Lidar_Angle(i); % Center of the Gaussian filter
    shifted_theta = mod(theta - deg2rad(center_angle) + pi, 2*pi) - pi; % Periodic adjustment
    gaussian_shape = 1 + exp(-shifted_theta.^2 / (6 * (deg2rad(gaussian_width))^2));
    
    % Convert Gaussian shape to Cartesian
    x_gaussian = gaussian_shape .* cos(theta);
    y_gaussian = gaussian_shape .* sin(theta);
    plot(x_gaussian, y_gaussian, 'black-', 'LineWidth', 1.5); % Gaussian plot
end

CenterPoint = plot(0, 0, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 10); % Red circle for start

% Formatting
axis equal;
grid on;
xlabel('X-axis');
ylabel('Y-axis');

hold off;

title('Feature Reduction Concept', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('X-axis', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Y-axis', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

% Adjust the axes to ensure ruler-like appearance
ax = gca; % Get current axes
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridColor = 'black'; % Grid color
ax.GridAlpha = 1.0; % Grid transparency
ax.LineWidth = 1.5; % Grid line thickness
ax.GridLineStyle = '-'; % Dashed grid lines
ax.GridAlpha = 0.1; % Transparency of grid lines

% % Enable axis ticks and labels
axis on;
set(ax, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

box on
axis(1.1*[-2 2 -2 2]); % Adjust axis limits if needed


