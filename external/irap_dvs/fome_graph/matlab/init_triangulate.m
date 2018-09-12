function feat2d = init_triangulate(feat2d,feat_old,feat_new,cameraParams,sc)
%%feat_old and feat_new = [x y idnumber inversedepth]
K = cameraParams.IntrinsicMatrix;

% triangulate initial with essential matrix
[E, inlier] = estimateEssentialMatrix(feat_old(:,1:2),feat_new(:,1:2),cameraParams,'Confidence',99,'MaxDistance',20);
ipt1 = feat_old(inlier,:);
ipt2 = feat_new(inlier,:);
% inlier_ratio = nnz(inlier)/length(inlier);
% fprintf("triangulation inlier : %2.2f%%\n",100*inlier_ratio);

% triangulation with inliers
[R_ess,t_ess] = relativeCameraPose(E,cameraParams,ipt1(:,1:2),ipt2(:,1:2));
for i=1:length(ipt1)
    if feat2d.idepth(feat2d.num == ipt1(i,3)) == 0 %if it is not triangulated yet, triangulate.
        norm_pt1 = K\[ipt1(i,1:2)'; 1];
        norm_pt2 = K\[ipt2(i,1:2)'; 1];
        a123 = R_ess*norm_pt1 - norm_pt2;
        t123 = sc*R_ess*t_ess';
        if (t123'*a123) < 0
            continue;
        end
        depth = sc*(t123'*a123) / (a123'*a123);
        feat2d.idepth(feat2d.num == ipt1(i,3)) = 1/depth;
        feat2d.cov(feat2d.num == ipt1(i,3)) = norm(norm_pt2'*E*norm_pt1);
    else %if it is already triangulated, update with smaller error
        olddepth = 1/feat2d.idepth(feat2d.num == ipt1(i,3));
        norm_pt1 = K\[ipt1(i,1:2)'; 1];
        norm_pt2 = K\[ipt2(i,1:2)'; 1];
        a123 = norm_pt2;
        t123 = R_ess*olddepth*norm_pt1 - sc*R_ess*t_ess';
        if (t123'*a123) < 0
            continue;
        end
        depth = (t123'*a123) / (a123'*a123);
        feat2d.idepth(feat2d.num == ipt1(i,3)) = 1/depth;
        feat2d.cov(feat2d.num == ipt1(i,3)) = norm(norm_pt2'*E*norm_pt1);
    end
end
% num3d = nnz(feat2d.idepth);
% fprintf("triangulated pts : %i\n",num3d);
end