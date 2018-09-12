function value = rank_eigenvalue(Moment,nth)
[~,S,~] = svd(Moment);
value = S(nth,nth);
end