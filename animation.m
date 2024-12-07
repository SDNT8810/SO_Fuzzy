h = Step_Counter ;
x = zeros(1,h);
y = zeros(1,h);
theta = zeros(1,h);
scale = 1;
for i = 1 : h 
    x(i) = scale * X(1,i) ; 
    y(i) = scale * X(3,i) ;  
    theta(i) = -X(5,i) ;  
end
scale = 1.7 * scale;
t = linspace(0,2*pi,100);
figure(1)
b = 0.18 ;

for i = 2 : 2 : h-1
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


    plot(0.25*sin(t)+(x(i)),0.25*cos(t)+y(i),':b','linewidth',1);
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
    
    line([x(i);4*(x(i+1))-(3*x(i))],[(y(i));4*(y(i+1))-3*(y(i))],'color',[0 0 0],'linewidth',2)

    axis([-scale scale*11 -scale scale*11])
    xlabel('X')
    ylabel('Y')
    hold off
    % pause(0.1);
    
end

hold on
plot(x,y) ;
hold off

