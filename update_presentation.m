%% Define some parameters
h = Step_Counter ;
x(h) = m2p * (X(1,h)); 
y(h) = m2p * (X(3,h));  
theta(h) = -X(5,h)*pi/180 ;  

skipframe_MF = 5;
skipframe_Lidar = 10;
skipframe_Lidar_line = 5;
skipframe_Debug = 10;
skipframe_map = 245;

map_x_max = max([x,m2p * X_g(1,1)]);
map_x_min = min([x,-2 * m2p]);
map_y_max = max([y,m2p * X_g(2,1)]);
map_y_min = min([y,-2 * m2p]);
map_margin = 1.2 * m2p;
map_axix = map_margin*[map_x_min map_x_max map_y_min map_y_max];

%% Lidar
if (max(floor(h/skipframe_Lidar)*skipframe_Lidar,1) == h)
    figure(1)
    subplot(2,3,3)
    Points360Plot = zeros(length(Points360),1);
    shiftIndexLidar = floor(X(5,h));
    shiftIndexLidar = (shiftIndexLidar + (360*(shiftIndexLidar<1)) + (-360*(shiftIndexLidar>360)));
    for jj = 1 : length(Points360)
        Points360Plot(jj) = Points360(1+length(Points360)-jj);
    end
    Points360Plot = circshift(flip(Points360), -shiftIndexLidar);
    polarplot(Lidar_Range-[Points360Plot;Points360Plot(1)]);
    % hold on;
    % Points360Plot = zeros(length(Points360_near),1);
    % for i = 1 : length(Points360_near)
    %     Points360Plot(h) = Points360_near(1+length(Points360_near)-i);
    % end
    % polarplot(Lidar_Range_near-[Points360Plot;Points360Plot(1)]);
    % hold off;
end

%% Preference_MF
if (max(floor(h/skipframe_MF)*skipframe_MF,1) == h)
    figure(1)
    subplot(2,3,6); 
    shiftIndexLidar = round(X(5,h)/Dist_MF_L2F);
    shiftIndexLidar = (shiftIndexLidar + (360*(shiftIndexLidar<1)) + (-360*(shiftIndexLidar>360)));
    Preference_MF_plot = circshift(Preference_MF, -shiftIndexLidar);
    polarplot([Preference_MF_plot; Preference_MF_plot(1)]);
    % DD = zeros(1,360); 
    % DD(round(Fuzzy_Local_Direction_ref+180)+1) = 1; 
    % hold on; 
    % polarplot(DD); 
    % polarplot([Preference_MF_near; Preference_MF_near(1)]); 
    % hold off;
end

%% Map
if (max(floor(h/skipframe_map)*skipframe_map,1) == h)
    % figure(2)
    subplot(2,3,[1,2,4,5])
    % axis(map_axix)
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
    
    % delete(lidar_circle);
    % lidar_circle = plot(c*sin(t)+(x(h)),c*cos(t)+y(h),':b','linewidth',0.8);
    lidar_line(360) = line([0; 0.1], [0.1; 0.1], 'color', 'blue');
    if (h == 360)
        disp('salam')
        for lidar_index = 1 : skipframe_Lidar_line : 360
                len = (Lidar_Range-Points360Plot(lidar_index))*m2p;
                lidar_line_mid(lidar_index) = line([x(h);x(h)+len*sind(lidar_index+90)],[y(h);y(h)+len*cosd(lidar_index+90)], 'color', 'blue', 'LineWidth', lw/10);
        end
    end
    delete(lidar_line);
    for lidar_index = 1 : skipframe_Lidar_line : 360
            len = (Lidar_Range-Points360Plot(lidar_index))*m2p;
            lidar_line(lidar_index) = line([x(h);x(h)+len*sind(lidar_index+90)],[y(h);y(h)+len*cosd(lidar_index+90)], 'color', 'blue', 'LineWidth', lw/10);
    end
    % plot(x,y) ;
    % title(['Time: ',num2str(T_s*Step_Counter), ', FLDR: ', num2str(Fuzzy_Local_Direction_ref)])
    % title(['Time: ',num2str(T_s*Step_Counter)], 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Arial', 'Color', 'black');
    title(['Time: ',num2str(T_s*Step_Counter)]);
    hLegend = legend([startPoint, targetPoint], {'Start Point', 'Target'}, 'FontSize', 24, 'Location', 'best');
    set(hLegend, 'FontName', 'Times New Roman');
    drawnow;% pause(0.001);
    hold off
end

%% Report Debug Mode
if ((Gazebo_Sim == 1) && (Debug_Mode == 1) && (max(floor(h/skipframe_Debug)*skipframe_Debug,1) == h))
    Debug_msg = 'Debug_Mode: ';
    Debug_msg = [Debug_msg, ' Heading = '];
    Debug_msg = [Debug_msg, num2str(Robot.Heading)];
    
    Debug_msg = [Debug_msg, ' , refang = '];
    Debug_msg = [Debug_msg, num2str(refang)];
    
    % Debug_msg = [Debug_msg, ' , argmaxang = '];
    % Debug_msg = [Debug_msg, num2str(argmaxang)];
    
    % Debug_msg = [Debug_msg, ' , FLDF = '];
    % Debug_msg = [Debug_msg, num2str(Fuzzy_Local_Direction_ref)];
    
    Debug_msg = [Debug_msg, ' , ntemp_w = '];
    Debug_msg = [Debug_msg, num2str(ntemp_w)];
    
    Debug_msg = [Debug_msg, ' , Goal_Direction = '];
    Debug_msg = [Debug_msg, num2str(Goal_Direction)];
    
    Debug_msg = [Debug_msg, ' , Goal_angle = '];
    Debug_msg = [Debug_msg, num2str(Goal_angle)];
    
    disp(Debug_msg)
end



