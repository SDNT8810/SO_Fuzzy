% Points360(1:100) = 5;
% Points360(101:200) = 3;
% Points360(201:300) = 1;
% Points360(301:360) = 2;
% 
Points360(1:10) = 2;
Points360(350:360) = 2;

Goal_Vector(:,Step_Counter) = [X_g(1,Step_Counter) - X(1,Step_Counter);X_g(3,Step_Counter) - X(2,Step_Counter)];
Goal_angle = atan2d(Goal_Vector(1,Step_Counter),Goal_Vector(1,Step_Counter));
Goal_Direction = Goal_angle - Robot.Heading;

MF_Lid = MF_Lidar(Points360);

bell_size = 70;
bell_coff = 3;
L = 180;

Phi2Goal = MF_Lidar_Angle - Goal_Direction;

SaturatedPHI2Goal = (Phi2Goal + (360*(Phi2Goal<-180)) + (-360*(Phi2Goal>180)));

weight_MF = gbellmf(SaturatedPHI2Goal,[bell_size, bell_coff, 0]);

Preference_MF = (weight_MF*0.7+0.3).*(1-MF_Lid);

% polarplot([Preference_MF; Preference_MF(1)]);



