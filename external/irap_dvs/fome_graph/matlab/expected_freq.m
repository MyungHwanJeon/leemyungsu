function freqmap = expected_freq(event_potential)
% add zero padding
ev = [zeros(244,2), [zeros(2,180);event_potential;zeros(2,180)], zeros(244,2)];
freqmap = zeros(240,180);

for i=1:240
    for j=1:180
        tmp = ev(i:i+4,j:j+4);
        if tmp(3,3)>0
            tmp2 = tmp(tmp>0);
        else
            tmp2 = tmp(tmp<0);
        end
        if isempty(tmp2)
            continue;
        end
        tmp3 = mean([abs(tmp2); tmp(3,3)]);
        freqmap(i,j) = tmp3;
    end
end
return