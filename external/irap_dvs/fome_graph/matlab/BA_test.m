K = [226.38018519795807 0 173.6470807871759;
    0 226.15002947047415 133.73271487507847;
    0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix', K);
ba_size = 10;
camposes = table([],{},{});
camposes.Properties.VariableNames = {'ViewId','Orientation','Location'};

for iter = 10:length(stats)-ba_size
    Viewnum = [];
    Pointuv = [];
    camposes.ViewId = [];
    camposes.Orientation = [];
    camposes.Location = [];
    feattrack = repmat(pointTrack(0,[0 0]),1, ba_size);
    
    %find features that is alive, from <iter> to <iter + ba_size - 1>
    [feat_live_num,feat_live_idx] = find_alive_feats_2d(feat2ds,iter,ba_size);
    [xyz,feat_live_num,feat_live_idx] = find_alive_feats_3d(feat3ds,iter,ba_size,feat_live_num,feat_live_idx);
    %construct camera pose
    for node = 1:ba_size
        state = stats{iter+node};
        camposes.ViewId(node) = uint32(node);
        camposes.Orientation{node} = eul2rotm(state(1:3));
        camposes.Location{node} = state(4:6);
        
        %construct tracking result
        [u, v] = uvfromidx(feat_live_idx{node});
        if node == 1
            for i=1:length(u)
                feattrack(i) = pointTrack(uint32(node),[u(i),v(i)]);
            end
        else
            for i=1:length(u)
                feattrack(i).ViewIds(end+1) = node;
                feattrack(i).Points(end+1,:) = [u(i), v(i)];
            end
        end
    end
    %bundle adjustment
    [xyz, camposes] = bundleAdjustment(xyz, feattrack, camposes, cameraParams,...
        'PointsUndistorted', true, 'MaxIterations', 200, 'RelativeTolerance', 1e-8);
    
    %update with result
    for node = 1:ba_size
        stats{iter+node} = [rot2rph(camposes.Orientation{node}), camposes.Location{node}];
        feat3ds{iter+node} = update_alive_feats_3d(feat3ds{iter+node},xyz,feat_live_num);
    end
    
end