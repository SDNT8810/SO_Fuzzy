
FuzzySysInputs = [Preference_MF;Goal_Direction];
% FuzzySysInputs = [rand(Num_MF_L2F,1)*1;Goal_Direction];

if RulesNum==0
    disp('*********** No Existed Rule: Add the First Rule ************')
    MeanMat(:,1) = FuzzySysInputs;
    VariMat(:,1) = ones(length(FuzzySysInputs),1)*Var0;
    W(1,1) = rand;
    RulesNum = 1;
    Antcs(1,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
else
    for i = 1:RulesNum
        Antcs(i) = Antc(FuzzySysInputs,MeanMat(:,i),VariMat(:,i),MF);
    end
    if max(Antcs)<0.5
        disp(['*********** No Enough Covering: Add a New Rule, RulesNum = ', num2str(RulesNum) , ' ************'])
        RulesNum = RulesNum+1;
        MeanMat(:,RulesNum) = FuzzySysInputs;
        VariMat(:,RulesNum) = ones(length(FuzzySysInputs),1)*Var0;
        W(RulesNum,1) = rand;
        Antcs(RulesNum,1) = Antc(FuzzySysInputs,MeanMat(:,RulesNum),VariMat(:,RulesNum),MF);
    end
end

% disp(['RulesNum : ',num2str(RulesNum)])
Fuzzy_Local_Direction_ref = ((180/pi) * (W'*Antcs) + 1*Goal_Direction)/2;
% Fuzzy_Local_Direction_ref = Goal_Direction;
% disp(['Fuzzy_Local_Direction_ref = ', num2str(Fuzzy_Local_Direction_ref)])


