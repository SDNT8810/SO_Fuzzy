
FuzzySysInputs = [Preference_MF;Goal_Direction];
% FuzzySysInputs = [rand(Num_MF_L2F,1)*1;Goal_Direction];

[~, argmaxang] = max(Preference_MF);
refang =  SaturatedPHI2Goal(argmaxang) + MF_Lidar_Angle(argmaxang);% +rand*360;
% refang = MF_Lidar_Angle'*Preference_MF / sum(Preference_MF);

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
    if (max(Antcs)<0.55 && RulesNum < Max_Number_of_Rulls)
        disp(['*********** No Enough Covering: Add a New Rule, RulesNum = ', num2str(RulesNum) , ' ************'])
        RulesNum = RulesNum+1;
        MeanMat(:,RulesNum) = FuzzySysInputs;
        VariMat(:,RulesNum) = ones(length(FuzzySysInputs),1)*Var0;
        W(RulesNum,1) = refang;
        Antcs(RulesNum,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
    end
end

% disp(['RulesNum : ',num2str(RulesNum)])

% Fuzzy_Local_Direction_ref = (W'*Antcs)/max(abs(sum(Antcs)),0.001);

Fuzzy_Local_Direction_ref = ((W'*Antcs)/sum(Antcs) + 2*Goal_Direction)/3;
% Fuzzy_Local_Direction_ref = Goal_Direction;
% disp(['Fuzzy_Local_Direction_ref = ', num2str(Fuzzy_Local_Direction_ref)])


