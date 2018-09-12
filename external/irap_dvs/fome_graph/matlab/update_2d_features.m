function feat_2d = update_2d_features(feat_2d,feat_2d_old,R,t,cameraParams,sc)
K = cameraParams.IntrinsicMatrix;
for i=1:length(feat_2d.num)
    if ismember(feat_2d.num(i),feat_2d_old.num)
        %tracked : triangulate again with R,t
        fromold = feat_2d_old.num==feat_2d.num(i);
        [u_new, v_new] = uvfromidx_undistort(feat_2d.idx(i));
        [u_old, v_old] = uvfromidx_undistort(feat_2d_old.idx(fromold));
        norm_old = K\[u_old; v_old; 1];
        norm_new = K\[u_new; v_new; 1];
        a123 = R*norm_old - norm_new;
        t123 = sc*R*t;
        if (t123'*a123) < 0
            %no optimal depth : do not update
            continue;
        end
        newdepth = (t123'*a123) / (a123'*a123);
        error = (a123'*a123)*newdepth - 2*sc*(t123'*a123)*newdepth + sc^2;
        feat_2d.idepth(i) = 1/newdepth;
        feat_2d.cov(i) = norm(error);
    else
        %not tracked : erase depth
        feat_2d.idepth(i) = 0;
    end
end
end