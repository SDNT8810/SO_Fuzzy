%% init
clc
clf

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
subplot(221)
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

% Formatting
axis equal;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
title('Feature Reduction Concept: 360 LiDAR Points to 13 Features');
hold off;

%% Flattened Gaussian with filled areas
% Parameters
subplot(2,2,[3,4])
hold on;
MF_Lid = 1-MF_Lid;
x_values = linspace(0, 360, 360); % X-axis points for visualization
MF_Lid(13) = 2 * MF_Lid(13); % Ensure circular behavior
MF_Lid(1)  = 2 * MF_Lid(1); % Ensure circular behavior

for i = 1:num_gaussians+1
    % Define Gaussian center and shape
    center_x = MF_Lidar_Angle(i); 
    gaussian_shape = exp(-(x_values - center_x).^2 / (2 * gaussian_width^2)); % Gaussian function

    % Define the maximum filled height (MF_Lid value)
    fill_height = MF_Lid(i);

    % Determine the fill region
    fill_x = x_values; % X-coordinates for the Gaussian
    fill_y = min(gaussian_shape, fill_height); % Limit the Gaussian by the fill height

    % Fill below the Gaussian to the MF_Lid value
    Filled = fill([fill_x, fliplr(fill_x)], [fill_y, zeros(size(fill_y))], ...
        [0.3 0.3 0.3], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
end

% Formatting
grid on;
xlabel('LiDAR Angle (degrees)');
ylabel('MF_Lid Value');
axis([0 360 0 1.05]); % Adjust axis limits if needed
hold off;

%% Plot Gaussian filters (Flattened on X-axis)

gaussian_width = Dist_MF_L2F / 4; % Gaussian width
x_values = linspace(0, 360, 360); % X-axis points for visualization

subplot(2,2,[3,4])
hold on;
for i = 1:num_gaussians+1
    center_x = MF_Lidar_Angle(i); % Center of the Gaussian filter on the X-axis
    gaussian_shape = exp(-(x_values - center_x).^2 / (2 * gaussian_width^2)); % Gaussian function
    Gaussian = plot(x_values, gaussian_shape, 'black', 'LineWidth', 1.5, 'LineStyle', '--'); % Plot Gaussian on X-axis
end

for i = 1 : 360
    len = (Lidar_Range-Points360Plot(i));
    blueLine = line([i;i],[0;len], 'color', [0 0.7 1], 'LineWidth', lw/4);
end
% Formatting
grid on;
% xlabel('LiDAR Angle (degrees)');
% ylabel('Gaussian Weight');
title('Flattened Gaussian Filters for Feature Reduction');
axis([0 360 0 1]); % Adjust axis limits if needed
hLegend = legend([blueLine, Filled, Gaussian], {'Input', 'Output', 'Gaussian'}, 'FontSize', 24, 'Location', 'best');
set(hLegend, 'FontName', 'Times New Roman');
hold off;

title('Flattened Gaussian Filters for Feature Reduction', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('LiDAR Angle (degrees)', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Gaussian Weight', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');


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




