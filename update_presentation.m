%% Define some parameters
h = Step_Counter ;
i = h ;
x(i) = m2p * (X(1,i)); 
y(i) = m2p * (X(3,i));  
theta(i) = -X(5,i) ;  

map_x_max = max([x,m2p * X_g(1,1)]);
map_x_min = min([x,-2 * m2p]);
map_y_max = max([y,m2p * X_g(2,1)]);
map_y_min = min([y,-2 * m2p]);
map_margin = 1.2 * m2p;
map_axix = map_margin*[map_x_min map_x_max map_y_min map_y_max];

%% Lidar
skipframe_Lidar = 1;
i = max(floor(h/skipframe_Lidar)*skipframe_Lidar,1);
if (i == h)
    figure(1)
    subplot(2,3,3)
    Points360Plot = zeros(length(Points360),1);
    for i = 1 : length(Points360)
        Points360Plot(i) = Points360(1+length(Points360)-i);
    end
    polarplot(Lidar_Range-[Points360Plot;Points360Plot(1)]);
    % polarplot([Points360Plot;Points360Plot(1)]);
end

%% path
% skipframe_Lidar = 1;
% i = max(floor(h/skipframe_Lidar)*skipframe_Lidar,1);
% if (i == h)
%     figure(1)
%     subplot(2,3,3)
%     plot(x,y,'LineWidth',1.2,'Color','b')
%     hold on
%     plot(X_g(1,1),X_g(2,1),'Marker','o','Color','r','LineWidth',2)
%     plot(X(1,1),X(3,1),'Marker','o','Color','g','LineWidth',2)
% 
% end

%% Preference_MF
skipframe_MF = 1;
i = max(floor(h/skipframe_MF)*skipframe_MF,1);
if (i == h)
    figure(1)
    subplot(2,3,6); 
    % th = 1:360; 
    % DD = zeros(1,360); DD(round(Fuzzy_Local_Direction_ref)+1) = 1; 
    polarplot([Preference_MF; Preference_MF(1)]); 
    % hold on; polarplot(th',DD'); hold off;
end

%% Map
skipframe_map = 3;
i = max(floor(h/skipframe_map)*skipframe_map,1);
b = 3 ;
c = Lidar_Range*m2p;

if (i == h)
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


    plot(c*sin(t)+(x(i)),c*cos(t)+y(i),':b','linewidth',1);


    % plot(0.2*sin(t)+XO1(i),0.2*cos(t)+YO1(i),':r','linewidth',1.5);
    % plot(0.2*sin(t)+XO2(i),0.2*cos(t)+YO2(i),':r','linewidth',1.5);
    % plot(0.2*sin(t)+XO3(i),0.2*cos(t)+YO3(i),'-r','linewidth',1.5);
    % plot(0.2*sin(t)+XO4(i),0.2*cos(t)+YO4(i),'-r','linewidth',1.5);
    % 
    % plot(0.4*sin(t)+XO1(i),0.4*cos(t)+YO1(i),':g','linewidth',1);
    % plot(0.4*sin(t)+XO2(i),0.4*cos(t)+YO2(i),':g','linewidth',1);
    % plot(0.4*sin(t)+XO3(i),0.4*cos(t)+YO3(i),':g','linewidth',1);
    % plot(0.4*sin(t)+XO4(i),0.4*cos(t)+YO4(i),':g','linewidth',1);

    % plot(0.2*sin(t)+XB(i),0.2*cos(t)+YB(i),'-g','linewidth',1.5);
    % plot(0.7*sin(t)+XB(i),0.7*cos(t)+YB(i),':g','linewidth',1);
%     line([x(i);9*(x(i+1))-(8*x(i))],[(y(i));9*(y(i+1))-8*(y(i))],'color',[0 0 0],'linewidth',2)

    % line([x(i);4*(x(i+1))-(3*x(i))],[(y(i));4*(y(i+1))-3*(y(i))],'color',[0 0 0],'linewidth',2)

    % axis([-scale scale*11 -scale scale*11])
    xlabel('X')
    ylabel('Y')
    % plot(x,y) ;
    title(['Time: ',num2str(Run_Timer(Step_Counter)), ', FLDR: ', num2str(Fuzzy_Local_Direction_ref)])
    drawnow; pause(0.1);
    hold off
end

%% Report
if ((Gazebo_Sim == 1) && (Debug_Mode == 1))
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



