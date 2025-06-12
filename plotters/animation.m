%% initializing
% clear; clf; clc; % close all;
clc

% Add functions directory to path
addpath('../functions');

%% Environmental Parameters

% subplot(121)
hImage = imshow(map', 'InitialMagnification', 'fit');

% Adjust transparency using 'AlphaData'
alphaValue = 0.3; % Set transparency level (0 = fully transparent, 1 = fully opaque)
set(hImage, 'AlphaData', alphaValue);

hold on;
startPoint = plot(m2p * X(1,1), m2p * X(3,1), 's', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', 'MarkerSize', 20); % Green marker for start
targetPoint = plot(m2p * X_g(1,1), m2p * X_g(2,1), 'p', 'MarkerFaceColor', 'r', 'MarkerSize', 40); % Red marker for goal
lidar_circle = plot(c*sin(t)+(X(1,1)),c*cos(t)+X(3,1),':b','linewidth',0.8);
delete(lidar_circle);

lidar_index = 360;
lidar_line(lidar_index) = line([0; 0.1], [0.1; 0.1], 'color', 'blue');
delete(lidar_line);
lw = 5;

% Get the size of the map
[rows, cols] = size(map);

% Define the rectangle around the map
rectangle('Position', [0.5, 0.5, rows, cols], 'EdgeColor', 'black', 'LineWidth', 5);

% Add title and axis labels
title(['Time: ', num2str(T_s * Step_Counter)], 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('X-axis (cm)', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Y-axis (cm)', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

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
set(ax, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

% Add legend
legend([startPoint, targetPoint], {'Start Point', 'Target'}, 'FontSize', FontSize, 'Location', 'best');

% load FuzzySystemLongMemory
tic
Step_Counter = 0;
c = 0.9 * c;

%% live Loop
while Step_Counter < 730

    Step_Counter = Step_Counter + 1;

    update_environment_parameters;
    Points360Plot = zeros(length(Points360),1);
    shiftIndexLidar = floor(X(5,h));
    shiftIndexLidar = (shiftIndexLidar + (360*(shiftIndexLidar<1)) + (-360*(shiftIndexLidar>360)));
    for jj = 1 : length(Points360)
        Points360Plot(jj) = Points360(1+length(Points360)-jj);
    end
    Points360Plot = circshift(flip(Points360), -shiftIndexLidar);

    h = Step_Counter ;
    x(h) = m2p * (X(1,h)); 
    y(h) = m2p * (X(3,h));  
    theta(h) = -X(5,h)*pi/180 ;  
    
    skipframe_MF = 5;
    skipframe_Lidar = 10;
    skipframe_Lidar_line = 5;
    skipframe_Debug = 10;
    skipframe_map = 30;
    
    map_x_max = max([x,m2p * X_g(1,1)]);
    map_x_min = min([x,-2 * m2p]);
    map_y_max = max([y,m2p * X_g(2,1)]);
    map_y_min = min([y,-2 * m2p]);
    map_margin = 1.2 * m2p;
    map_axix = map_margin*[map_x_min map_x_max map_y_min map_y_max];
   
    % Map
    if (max(floor(h/skipframe_map)*skipframe_map,1) == h)
        hold on
    
        xw1=x(h)+b*sin(theta(h)-pi/2)-b*sin(theta(h));
        yw1=y(h)+b*cos(theta(h)-pi/2)-b*cos(theta(h));
        xw2=x(h)+b*sin(theta(h)-pi/2)+b*sin(theta(h));
        yw2=y(h)+b*cos(theta(h)-pi/2)+b*cos(theta(h));
        xw3=x(h)-b/2*sin(theta(h)-pi/2)+b*sin(theta(h));
        yw3=y(h)-b/2*cos(theta(h)-pi/2)+b*cos(theta(h));
        xw4=x(h)-b*sin(theta(h)-pi/2)+b/2*sin(theta(h));
        yw4=y(h)-b*cos(theta(h)-pi/2)+b/2*cos(theta(h));
        xw5=x(h)-b*sin(theta(h)-pi/2)-b/2*sin(theta(h));
        yw5=y(h)-b*cos(theta(h)-pi/2)-b/2*cos(theta(h));
        xw6=x(h)-b/2*sin(theta(h)-pi/2)-b*sin(theta(h));
        yw6=y(h)-b/2*cos(theta(h)-pi/2)-b*cos(theta(h));
    
        line([xw1;xw2],[yw1;yw2],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        line([xw2;xw3],[yw2;yw3],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        line([xw3;xw4],[yw3;yw4],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        line([xw4;xw5],[yw4;yw5],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        line([xw5;xw6],[yw5;yw6],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        line([xw6;xw1],[yw6;yw1],'color',[0.5 0.5 0.5], 'LineWidth', lw)
        lidar_line(360) = line([0; 0.1], [0.1; 0.1], 'color', 'blue');
        
        delete(lidar_line);
        for lidar_index = 1 : skipframe_Lidar_line : 360
                len = (Lidar_Range-Points360Plot(lidar_index))*m2p;
                lidar_line(lidar_index) = line([x(h);x(h)+len*sind(lidar_index+90)],[y(h);y(h)+len*cosd(lidar_index+90)], 'color', 'blue', 'LineWidth', lw/10);
        end
        title(['Time: ',num2str(T_s*Step_Counter)]);
        hLegend = legend([startPoint, targetPoint], {'Start Point', 'Target'}, 'FontSize', FontSize, 'Location', 'best');
        set(hLegend, 'FontName', 'Times New Roman');
        drawnow;% pause(0.001);
        hold off
    end
end

%% plot path
hold on
plot_path = plot(m2p * (X(1,1:h)),m2p * (X(3,1:h)), 'LineWidth', 2, 'LineStyle', '--', 'color', 'black');
hLegend = legend([startPoint, targetPoint, plot_path], {'Start Point', 'Target', 'Path'}, 'FontSize', FontSize, 'Location', 'best');
set(hLegend, 'FontName', 'Times New Roman');
% delete(lidar_line);
