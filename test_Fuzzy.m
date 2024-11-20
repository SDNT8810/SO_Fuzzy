
clear;clc
MeanMat = [];
VariMat = [];
Var0 = 1;
W = [];
RulesNum = 0;
MF = @(X,l,i,M,S) gaussmf(X,[S, M]);
function out = Antc(X,l,M,S,MF)
  for i=1:length(X)
    Ant(i) = MF(X(i),l,i,M(i,l),S(i,l));
  end
  out = prod(Ant);
end

for k = 1:50
  X = rand(3,1)*3
  
  if RulesNum==0
    disp('*********** No Existed Rule: Add the First Rule ************')
    MeanMat(:,1) = X;
    VariMat(:,1) = ones(length(X),1)*Var0;
    W(1,1) = rand;
    RulesNum = 1;
    Antcs(1,1) = Antc(X,1,MeanMat,VariMat,MF);
  else
    for l = 1:RulesNum
      Antcs(l) = Antc(X,l,MeanMat,VariMat,MF);
    end
    if max(Antcs)<0.5
      disp('*********** No Enough Covering: Add a New Rule ************')
      RulesNum = RulesNum+1;
      MeanMat(:,RulesNum) = X;
      VariMat(:,RulesNum) = ones(length(X),1)*Var0;
      W(RulesNum,1) = rand;
      Antcs(RulesNum,1) = Antc(X,RulesNum,MeanMat,VariMat,MF);    
    end
  end

  disp(['RulesNum : ',num2str(RulesNum)])
  disp(['Max Covering : ', num2str(max(Antcs))])

  Fuzzy_Local_Direction_ref = W'*Antcs;
  disp(['Fuzzy_Local_Direction_ref = ', num2str(Fuzzy_Local_Direction_ref)])

end


