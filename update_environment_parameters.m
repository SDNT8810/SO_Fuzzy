
if (Gazebo_Sim == 1)
    Points360 = LaserSub.LatestMessage.ranges;
    Points360(Points360==Inf) = Lidar_Range;
else
    Points360 = Read_Lidar(X(:,Step_Counter), m2p, Lidar_Range, map_local);
    Points360_near = Read_Lidar(X(:,Step_Counter), m2p, Lidar_Range_near, map_local);
end
Points360 = Lidar_Range - Points360;
Points360_near = Lidar_Range_near - Points360_near;

Goal_Vector(:,Step_Counter) = [X_g(1,Step_Counter) - X(1,Step_Counter);X_g(2,Step_Counter) - X(3,Step_Counter)];
Goal_angle = atan2d(Goal_Vector(2,Step_Counter),Goal_Vector(1,Step_Counter));
Goal_Direction = Goal_angle - Robot.Heading;
Goal_Direction = (Goal_Direction + (360*(Goal_Direction<-180)) + (-360*(Goal_Direction>180)));
MF_Lid = MF_Lidar(Points360);
MF_Lid_near = MF_Lidar(Points360_near);

Phi2Goal = MF_Lidar_Angle - Goal_Direction;

SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));

weight_MF = gbellmf(SaturatedPHI2Goal,[bell_size, bell_coff, 0]);

Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);
Preference_MF_near = (weight_MF*0.7+0.3).*(1-MF_Lid_near);
% N_MF_Lid = Lidar_Range./min(Lidar_Range,max(MF_Lid,0.001));
% Preference_MF = (weight_MF*0.7+0.3).* N_MF_Lid;
Preference_MF = Preference_MF / max(Preference_MF);
Preference_MF_near = Preference_MF_near / max(Preference_MF_near);


