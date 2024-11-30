
% Points360 = Read_Lidar(X(:,Step_Counter), X_g(:,Step_Counter), Lidar_Range, map);
% figure(2)
% polarplot(5-Points360);
% pause(0.05)

% Points360(1:100) = 5;
% Points360(101:200) = 3;
% Points360(201:300) = 1;
% Points360(301:360) = 2;
% 
% Points360(1:10) = 0;
% Points360(350:360) = 0;

if (Gazebo_Sim == 1)
    Points360 = LaserSub.LatestMessage.ranges;
    Points360(Points360==Inf) = Lidar_Range;
    Points360 = Lidar_Range - Points360;
else
    % px = rl + X(1,Step_Counter) * m2p;
    % py = rl + X(3,Step_Counter) * m2p;
    Points360 = Read_Lidar(X(:,Step_Counter), m2p, Lidar_Range, map_local);
    Points360 = Lidar_Range - Points360;
end

Goal_Vector(:,Step_Counter) = [X_g(1,Step_Counter) - X(1,Step_Counter);X_g(2,Step_Counter) - X(3,Step_Counter)];
Goal_angle = atan2d(Goal_Vector(2,Step_Counter),Goal_Vector(1,Step_Counter));
Goal_Direction = Goal_angle - Robot.Heading;
Goal_Direction = (Goal_Direction + (360*(Goal_Direction<-180)) + (-360*(Goal_Direction>180)));
MF_Lid = MF_Lidar(Points360);

Phi2Goal = MF_Lidar_Angle - Goal_Direction;

SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));

weight_MF = gbellmf(SaturatedPHI2Goal,[bell_size, bell_coff, 0]);

Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);


dist2goal(X,X_g(:,Step_Counter));
