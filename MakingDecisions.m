MF_Lid = MF_Lidar(Points360);

Goal_Vector(:,Step_Counter) = [X_g(Step_Counter,1) - X(Step_Counter,1);X_g(Step_Counter,3) - X(Step_Counter,2)];
Goal_angle = 180 * atan2(Goal_Vector(1,Step_Counter),Goal_Vector(1,Step_Counter)) / pi;

bell_size = 70;
bell_coff = 3;

weighting_range = Goal_angle - 360 : Goal_angle + 360;
weightingMF = gbellmf(weighting_range,[bell_size, bell_coff, Goal_angle]);

weighting_range = Goal_angle - 360 : Goal_angle + 360;
weightingMF = weightingMF + gbellmf(weighting_range,[bell_size, bell_coff, Goal_angle + 360]);

plweighting_range = Goal_angle - 360 : Goal_angle + 360;
weightingMF = weightingMF + gbellmf(weighting_range,[bell_size, bell_coff, Goal_angle - 360]);


MF_Lid(1) = 0
weightingMF = 1 - 0.5 * (1 + MF_Lid(1)) * (1 - weightingMF); 
plot(weighting_range/360,weightingMF)
axis equal
MF_Weghted = MF_Lid;






