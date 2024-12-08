
% FuzzySysInputs = [Preference_MF;Goal_Direction/360;dist2goal(X,X_g(:,Step_Counter))/2;Robot.Heading/180];
FuzzySysInputs = [Preference_MF_near;Preference_MF;Goal_Direction/180;dist2goal(X,X_g(:,Step_Counter));Robot.Heading/180];

[~, argmaxang] = max(Preference_MF);
saturated_MF_Lidar_Angle = (MF_Lidar_Angle + (360*(MF_Lidar_Angle<-180)) + (-360*(MF_Lidar_Angle>180)));

refang =  SaturatedPHI2Goal(argmaxang) + saturated_MF_Lidar_Angle(argmaxang);% +rand*360;
refang = refang / 180;
V = (2*V + 0.3 * (1-MF_Lid_near(argmaxang)))/3;
% refang = MF_Lidar_Angle'*Preference_MF / sum(Preference_MF);

%% add New Rull
if RulesNum==0
    disp('*********** No Existed Rule: Add the First Rule ************')
    MeanMat(:,1) = FuzzySysInputs;
    VariMat(:,1) = ones(length(FuzzySysInputs),1)*Var0;
    W(1,1) = refang;
    RulesNum = 1;
    age_of_Rulls(RulesNum,1) = 0;
    Antcs(1,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
else
    Antcs = zeros(RulesNum,1);
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
        age_of_Rulls(RulesNum,1) = 0;
    end
end

%% Eliminate Rulls
if RulesNum > 1
    % check age Step_Counter
    % max_age = max(max_age,Step_Counter);
    Not_Fired_Rulls = (Antcs<min_gamma);
    age_of_Rulls = age_of_Rulls .* Not_Fired_Rulls + Not_Fired_Rulls;
    valid_rulls = (age_of_Rulls < max_age);
    if sum(valid_rulls) < RulesNum
        disp(['############################### Too Old Rull, RulesNum = ', num2str(RulesNum) , ' ************'])
        RulesNum = sum(valid_rulls);
        MeanMat = MeanMat(:,valid_rulls');
        VariMat = VariMat(:,valid_rulls');
        Antcs = Antcs(valid_rulls,:);
        W = W(valid_rulls,:);
        age_of_Rulls = age_of_Rulls(valid_rulls,:);
        Not_Fired_Rulls = Not_Fired_Rulls(valid_rulls,:);
    end
end

if RulesNum > 2
    % check similarity
    i = 0;
    while i < RulesNum - 1
        i = i + 1;
        diff_input = MeanMat(:,i+1:end) - MeanMat(:,i);
        norm_diff_input = zeros(RulesNum - i,1);
        for j = 1 : RulesNum - i
            norm_diff_input(j) = sum(diff_input(:,j).^2);
        end
        similarInputs = (norm_diff_input < min_similarity);
        if (sum(similarInputs) > 0)
            disp(['@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Eliminate Similar Rull, RulesNum = ', num2str(RulesNum) , ' ************'])
            unique_Inputs = ([ones(i,1);1 - similarInputs] == 1);
            RulesNum = RulesNum - sum(similarInputs);
            MeanMat = MeanMat(:,unique_Inputs);
            VariMat = VariMat(:,unique_Inputs);
            Antcs = Antcs(unique_Inputs,:);
            W = W(unique_Inputs,:);
            age_of_Rulls = age_of_Rulls(unique_Inputs,:);
            Not_Fired_Rulls = Not_Fired_Rulls(unique_Inputs,:);
        end

    end

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


