function feat3d = update_alive_feats_3d(feat3d,xyz,feat_live_num)

for i = 1:length(feat_live_num)
    if ismember(feat_live_num(i),feat3d.id)
    	loc = (feat_live_num(i)==feat3d.id);
        feat3d.xyz(loc) = xyz(i);
    end
end

end