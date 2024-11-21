MF_Lid = MF_Lidar(Points360);






Goal_Vector(:,Step_Counter) = [X_g(Step_Counter,1) - X(Step_Counter,1);X_g(Step_Counter,3) - X(Step_Counter,2)];
Goal_angle = atan2d(Goal_Vector(1,Step_Counter),Goal_Vector(1,Step_Counter))
Goal_Direction = Goal_angle - Robot.Heading

bell_size = 70;
bell_coff = 3;
L = 180;

% weighting_range = -L : +L;
% weightingMF = gbellmf(weighting_range,[bell_size, bell_coff, 0]);

Phi = MF_Lidar_Angle - Goal_Direction

PHI = Phi + (360*(Phi<-180)) + (-360*(Phi>180))

weight_F = gbellmf(PHI,[bell_size, bell_coff, 0])

Preference_MF = (weight_F*0.7+0.3)'.*(1-MF_Lid)

polarplot([Preference_MF; Preference_MF(1)]);







