figure(3)

Dist_MF_L2F = 30;
MF_Lidar_Angle = (0:Dist_MF_L2F:359)';

Num_MF_L2F = 360/Dist_MF_L2F;
Membership_Lidar = zeros(2*Dist_MF_L2F+1 , Num_MF_L2F);
MF_Lidar_ = zeros(Num_MF_L2F, 1);
MF_L2F(:,1) = gaussmf(-Dist_MF_L2F:Dist_MF_L2F , [(Dist_MF_L2F/4)/sqrt(-2*log(0.5)), 0]); 
Lidar_Augmented = @(x) [x(end-Dist_MF_L2F:end,1); x; x(1:Dist_MF_L2F,1)];
Max_Lidar = sum(Lidar_Range * MF_L2F);
MF_Lidar = @(Points360) Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar);


for i = 1 : Num_MF_L2F
    gussian(i) = []
    plot(MF_L2F)
end