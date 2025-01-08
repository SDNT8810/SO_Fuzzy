%% init
clc
figure(2)
clf
subplot(221)

Goal_angle = 220;

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

%% Plot Gaussian filters (Mexican hat)
gaussian_width = Dist_MF_L2F / 4; % Gaussian width
for i = 1:num_gaussians
    center_angle = MF_Lidar_Angle(i); % Center of the Gaussian filter
    shifted_theta = mod(theta - deg2rad(center_angle) + pi, 2*pi) - pi; % Periodic adjustment
    gaussian_shape = 0 + exp(-shifted_theta.^2 / (6 * (deg2rad(gaussian_width))^2));
    
    % Convert Gaussian shape to Cartesian
    x_gaussian = gaussian_shape .* cos(theta);
    y_gaussian = gaussian_shape .* sin(theta);
    Guassians = plot(x_gaussian, y_gaussian, 'black-', 'LineWidth', 1.5); % Gaussian plot
end

%% fill
% Parameters
theta_circle = linspace(0, 2*pi, 500); % Circle outline
r_circle = 1; % Radius of the base circle

% Draw the reference circle
reference_circle = plot(r_circle * cos(theta_circle), r_circle * sin(theta_circle), 'k--', 'LineWidth', 0.7);

% Define angles for each Gaussian and map points
num_angles = 360; % Total points for LiDAR
theta = linspace(0, 2*pi, num_angles); % Circular angles
x_values = theta; % X-axis values for Gaussian calculation

% Loop over each Gaussian
for i = 1:num_gaussians
    % Define the Gaussian parameters
    center_theta = deg2rad(MF_Lidar_Angle(i)); % Center of the Gaussian in radians
    gaussian_shape = exp(-(x_values - center_theta).^2 / (6 * deg2rad(gaussian_width)^2)); % Gaussian function
     % 0 + exp(-shifted_theta.^2 / (6 * (deg2rad(gaussian_width))^2));
    % Determine the fill region height
    fill_height = Lidar_Range - MF_Lid(i); % Scaled radius based on MF_Lid value
    
    % Convert Gaussian to polar coordinates
    r_filled = min(gaussian_shape, fill_height); % Filled Gaussian radius
    x_filled = r_filled .* cos(x_values); % X-coordinates for filled region
    y_filled = r_filled .* sin(x_values); % Y-coordinates for filled region

    % Fill the Gaussian on the circle
    Inputs = fill([x_filled, 0], [y_filled, 0], [0.3 0.3 0.3], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % Plot the Gaussian outline
    r_gaussian = gaussian_shape; % Full Gaussian radius
    x_gaussian = r_gaussian .* cos(x_values); % X-coordinates for outline
    y_gaussian = r_gaussian .* sin(x_values); % Y-coordinates for outline
    % plot(x_gaussian, y_gaussian, 'black-', 'LineWidth', 1.5);
end

CenterPoint = plot(0, 0, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 10); % Red circle for start

center_angle = Goal_angle; % Center of the Gaussian filter
GoalDir = quiver(0, 0, R*cosd(center_angle), R*sind(center_angle), 0, 'b', 'LineWidth', 3, 'MaxHeadSize', .35);

%% Formatting
axis equal;
hold off;

title('Input weighting representation based on robot heading', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
% xlabel('X-axis', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
% ylabel('Y-axis', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

% Adjust the axes to ensure ruler-like appearance
ax = gca; % Get current axes
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridColor = 'black'; % Grid color
ax.GridAlpha = 1.0; % Grid transparency
ax.LineWidth = 1.5; % Grid line thickness
ax.GridLineStyle = '-'; % Dashed grid lines
ax.GridAlpha = 0.04; % Transparency of grid lines

% % Enable axis ticks and labels
axis on;
set(ax, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

box on
axis([-1.1 1.1 -1.1 1.6]); % Adjust axis limits if needed

hLegend = legend([GoalDir, Inputs, Weighting, reference_circle], {'Robot heading', 'Inputs', 'Weight', 'Reference circle'}, 'FontSize', FontSize, 'Location', 'best');
set(hLegend, 'FontName', 'Times New Roman');




%% Flattened Gaussian with filled areas
% Parameters
subplot(2,2,3)
hold on;
MF_Lid_ = Lidar_Range-MF_Lid;
x_values = linspace(0, 360, 360); % X-axis points for visualization
% MF_Lid_(13) = 2 * MF_Lid_(13); % Ensure circular behavior
% MF_Lid_(1)  = 2 * MF_Lid_(1); % Ensure circular behavior

center_gaussian = Goal_angle; % Center of the Gaussian
gaussian_width = 25; % Width of the Gaussian
num_intervals = 13; % Number of intervals (squares)
interval_size = 360 / num_intervals; % Interval width on the X-axis

% Generate Gaussian
gaussian_shape = ( 0.3 + 0.7 * exp(-(x_values - center_gaussian).^2 / (2 * gaussian_width^2)));

% Plot MF_Lid_ as squares
hold on;
for i = 1:num_intervals
    x_start = (i - 1) * interval_size;
    x_end = i * interval_size;
    y_height = MF_Lid_(i);
    interval_x = x_values(x_values >= x_start & x_values <= x_end); % X within the interval
    % blueLine = fill([x_start, x_end, x_end, x_start], [0, 0, y_height, y_height], ...
    %     [0.3 0.3 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Blue squares

    blueLine = fill([interval_x, fliplr(interval_x)], ...
        [y_height+zeros(size(interval_x)), zeros(size(interval_x))], ...
        [0.3 0.3 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Blue squares
end

% Plot Gaussian shape
Gaussian = plot(x_values, gaussian_shape, 'k-', 'LineWidth', 2.5); % Black Gaussian line

% Compute and fill weighted result
for i = 1:num_intervals
    x_start = (i - 1) * interval_size;
    x_end = i * interval_size;
    interval_x = x_values(x_values >= x_start & x_values <= x_end); % X within the interval
    interval_gaussian = zeros(size(interval_x)) + mean(gaussian_shape(x_values >= x_start & x_values <= x_end)); % Gaussian in interval
    weighted_result = (interval_gaussian * MF_Lid_(i)); % Element-wise minimum
    Filled = fill([interval_x, fliplr(interval_x)], ...
         [weighted_result, zeros(size(weighted_result))], ...
         [1 0.3 0.3], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'LineStyle', '--'); % Red squares         
    Node_2_Output(i) = MF_Lid_(i) * mean(gaussian_shape(x_values >= x_start & x_values <= x_end));
end

%% Formatting
grid on;
axis([0 360 0 1.1]); % Adjust axis limits if needed
hLegend = legend([blueLine, Filled, Gaussian], {'Input', 'Output', 'Weighting'}, 'FontSize', FontSize, 'Location', 'best');
set(hLegend, 'FontName', 'Times New Roman');
hold off;

title('Gaussian filter for weighting based on robot heading', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('Angle (degrees)', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Input/Output of Node 2', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

% Adjust the axes to ensure ruler-like appearance
ax = gca; % Get current axes
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridColor = 'black'; % Grid color
ax.GridAlpha = 1.0; % Grid transparency
ax.LineWidth = 1.5; % Grid line thickness
ax.GridLineStyle = '-'; % Dashed grid lines
ax.GridAlpha = 0.01; % Transparency of grid lines

% Enable axis ticks and labels
axis on;
set(ax, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

box on

Node_2_Input = MF_Lid_;


