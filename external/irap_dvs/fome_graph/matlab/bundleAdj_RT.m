function [feat3ds, stats] = bundleAdj_RT(feat2ds, feat3ds, stats, cameraParams,ba_size)
camposes = table([],{},{});
camposes.Properties.VariableNames = {'ViewId','Orientation','Location'};

now = length(stats);
camposes.ViewId = [];
camposes.Orientation = [];
camposes.Location = [];
feattrack = repmat(pointTrack(0,[0 0]),1, ba_size);

%find features that is alive, from <now - ba_size + 1> to <now>
[feat_live_num,feat_live_idx] = find_alive_feats_2d(feat2ds, now-ba_size+1,ba_size);
[xyz,feat_live_num,feat_live_idx] = find_alive_feats_3d(feat3ds,now-ba_size+1,ba_size,feat_live_num,feat_live_idx);
%construct camera pose
for node = 1:ba_size
    state = stats{now - ba_size +node};
    camposes.ViewId(node) = uint32(node);
    camposes.Orientation{node} = eul2rotm(state(1:3));
    camposes.Location{node} = state(4:6);

    %construct tracking result
    feattrack = update_feattrack(feattrack,feat_live_idx,node);
end
%bundle adjustment
[xyz, camposes] = bundleAdjustment(xyz, feattrack, camposes, cameraParams,...
    'PointsUndistorted', true, 'Maxiterations', 100, 'AbsoluteTolerance', 5);

%update with result
for node = 1:ba_size
    stats{now - ba_size + node} = [rot2rph(camposes.Orientation{node})', camposes.Location{node}];
end
feat3ds{now} = update_alive_feats_3d(feat3ds{now},xyz,feat_live_num);

end