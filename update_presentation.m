%% Define some parameters
h = Step_Counter ;
i = h ;
x(i) = m2p * (X(1,i)); 
y(i) = m2p * (X(3,i));  
theta(i) = -X(5,i)*pi/180 ;  

skipframe_MF = 5;
skipframe_Lidar = 5;
skipframe_Debug = 10;
skipframe_map = 10;

map_x_max = max([x,m2p * X_g(1,1)]);
map_x_min = min([x,-2 * m2p]);
map_y_max = max([y,m2p * X_g(2,1)]);
map_y_min = min([y,-2 * m2p]);
map_margin = 1.2 * m2p;
map_axix = map_margin*[map_x_min map_x_max map_y_min map_y_max];

%% Lidar
i = max(floor(h/skipframe_Lidar)*skipframe_Lidar,1);
if (i == h)
    figure(1)
    subplot(2,3,3)
    Points360Plot = zeros(length(Points360),1);
    shiftIndexLidar = floor(X(5,i));
    shiftIndexLidar = (shiftIndexLidar + (360*(shiftIndexLidar<1)) + (-360*(shiftIndexLidar>360)));
    for jj = 1 : length(Points360)
        Points360Plot(jj) = Points360(1+length(Points360)-jj);
    end
    Points360Plot = circshift(flip(Points360), -shiftIndexLidar);
    polarplot(Lidar_Range-[Points360Plot;Points360Plot(1)]);
    % hold on;
    % Points360Plot = zeros(length(Points360_near),1);
    % for i = 1 : length(Points360_near)
    %     Points360Plot(i) = Points360_near(1+length(Points360_near)-i);
    % end
    % polarplot(Lidar_Range_near-[Points360Plot;Points360Plot(1)]);
    % hold off;
end

%% Preference_MF
i = max(floor(h/skipframe_MF)*skipframe_MF,1);
if (i == h)
    figure(1)
    subplot(2,3,6); 
    th = 1:360; 
    DD = zeros(1,360); 
    shiftIndexLidar = round(X(5,i)/Dist_MF_L2F);
    shiftIndexLidar = (shiftIndexLidar + (360*(shiftIndexLidar<1)) + (-360*(shiftIndexLidar>360)));
    DD(round(Fuzzy_Local_Direction_ref+180)+1) = 1; 
    Preference_MF_plot = circshift(Preference_MF, -shiftIndexLidar);
    polarplot([Preference_MF_plot; Preference_MF_plot(1)]); 
    hold on; 
    polarplot(th,DD); 
    % polarplot([Preference_MF_near; Preference_MF_near(1)]); 
    hold off;
end

%% Map
i = max(floor(h/skipframe_map)*skipframe_map,1);
% c_near = Lidar_Range_near*m2p;

if (i == h) && (i > 1)
    % figure(2)
    subplot(2,3,[1,2,4,5])
    % axis(map_axix)
    hold on

    xw1=x(i)+b*sin(theta(i)-pi/2)-b*sin(theta(i));
    yw1=y(i)+b*cos(theta(i)-pi/2)-b*cos(theta(i));
    xw2=x(i)+b*sin(theta(i)-pi/2)+b*sin(theta(i));
    yw2=y(i)+b*cos(theta(i)-pi/2)+b*cos(theta(i));
    xw3=x(i)-b/2*sin(theta(i)-pi/2)+b*sin(theta(i));
    yw3=y(i)-b/2*cos(theta(i)-pi/2)+b*cos(theta(i));
    xw4=x(i)-b*sin(theta(i)-pi/2)+b/2*sin(theta(i));
    yw4=y(i)-b*cos(theta(i)-pi/2)+b/2*cos(theta(i));
    xw5=x(i)-b*sin(theta(i)-pi/2)-b/2*sin(theta(i));
    yw5=y(i)-b*cos(theta(i)-pi/2)-b/2*cos(theta(i));
    xw6=x(i)-b/2*sin(theta(i)-pi/2)-b*sin(theta(i));
    yw6=y(i)-b/2*cos(theta(i)-pi/2)-b*cos(theta(i));

    line([xw1;xw2],[yw1;yw2],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    line([xw2;xw3],[yw2;yw3],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    line([xw3;xw4],[yw3;yw4],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    line([xw4;xw5],[yw4;yw5],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    line([xw5;xw6],[yw5;yw6],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    line([xw6;xw1],[yw6;yw1],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
    
    delete(lidar_circle);
    lidar_circle = plot(c*sin(t)+(x(i)),c*cos(t)+y(i),':b','linewidth',0.8);
    % plot(c_near*sin(t)+(x(i)),c_near*cos(t)+y(i),':r','linewidth',0.8);

    xlabel('X')
    ylabel('Y')
    % plot(x,y) ;
    title(['Time: ',num2str(T_s*Step_Counter), ', FLDR: ', num2str(Fuzzy_Local_Direction_ref)])
    drawnow;% pause(0.001);
    hold off
end

%% Report Debug Mode
i = max(floor(h/skipframe_Debug)*skipframe_Debug,1);
if ((Gazebo_Sim == 1) && (Debug_Mode == 1) && (i == h))
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



