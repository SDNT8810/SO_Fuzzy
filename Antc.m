function out = Antc(X,M,S,MF)
    Ant = ones(length(X),1);
    for i=1:length(X)
        Ant(i) = MF(X(i),M(i),S(i));
    end
    out = prod(Ant);
end