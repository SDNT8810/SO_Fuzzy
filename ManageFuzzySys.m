
FuzzySysInputs = [Preference_MF;Goal_Direction];

[~, argmaxang] = max(Preference_MF);
saturated_MF_Lidar_Angle = (MF_Lidar_Angle + (360*(MF_Lidar_Angle<-180)) + (-360*(MF_Lidar_Angle>180)));

refang =  SaturatedPHI2Goal(argmaxang) + saturated_MF_Lidar_Angle(argmaxang);% +rand*360;
refang = refang / 180;
% refang = MF_Lidar_Angle'*Preference_MF / sum(Preference_MF);

%% add New Rull
if RulesNum==0
    disp('*********** No Existed Rule: Add the First Rule ************')
    MeanMat(:,1) = FuzzySysInputs;
    VariMat(:,1) = ones(length(FuzzySysInputs),1)*Var0;
    W(1,1) = refang;
    RulesNum = 1;
    Antcs(1,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
else
    for i = 1:RulesNum
        Antcs(i) = Antc(FuzzySysInputs,MeanMat(:,i),VariMat(:,i),MF);
    end
    if (max(Antcs)<gamma && RulesNum < Max_Number_of_Rulls)
        disp(['*********** No Enough Covering: Add a New Rule, RulesNum = ', num2str(RulesNum) , ' ************'])
        RulesNum = RulesNum+1;
        MeanMat(:,RulesNum) = FuzzySysInputs;
        VariMat(:,RulesNum) = ones(length(FuzzySysInputs),1)*Var0;
        W(RulesNum,1) = refang;
        Antcs(RulesNum,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
    end
end

%% Eliminate Rulls
if RulesNum > 1
    % check similarity
    
end

% disp(['RulesNum : ',num2str(RulesNum)])
W = (W + (2*(W<-1)) + (-2*(W>1)));


if sum(Antcs) == 0
    Fuzzy_Local_Direction_ref = 0;
else
    Fuzzy_Local_Direction_ref = 180 * (W'*Antcs)/abs(sum(Antcs));
end

% Fuzzy_Local_Direction_ref = ((W'*Antcs)/sum(Antcs) + 2*Goal_Direction)/3;
% Fuzzy_Local_Direction_ref = Goal_Direction;
% disp(['Fuzzy_Local_Direction_ref = ', num2str(Fuzzy_Local_Direction_ref)])


