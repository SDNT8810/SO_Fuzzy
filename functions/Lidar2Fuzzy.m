function o = Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar)
  Y = Lidar_Augmented(Points360);
  for j = 1:Num_MF_L2F
    y = Y((j-1)*Dist_MF_L2F+1: (j+1)*Dist_MF_L2F+1);
    Membership_Lidar(:,j) = MF_L2F.*y;
    MF_Lidar_(j,1) = sum(Membership_Lidar(:,j));
  end
  o = MF_Lidar_/Max_Lidar;
end