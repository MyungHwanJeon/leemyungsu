function se3 = make_guess_se3_svd(new_3d_points,old_3d_points)

inlier = false(length(new_3d_points(:,:)),1);
inlieridx = [];
for l=1:1:length(inlier)
     if (new_3d_points(3,l) > 1e-06) && (~isnan(new_3d_points(3,l)))
        inlier(l) = true;
        inlieridx = [inlieridx; l];
    end
end
if (nnz(inlier) < 8)
    se3 = rt2tr(eul2rotm([0 0 0]),[0 0 0]);
    return;
end

%find optimal value
centroid_old = [mean(old_3d_points(1,inlier)) mean(old_3d_points(2,inlier)) mean(old_3d_points(3,inlier))];
centroid_new = [mean(new_3d_points(1,inlier)) mean(new_3d_points(2,inlier)) mean(new_3d_points(3,inlier))];
H = (old_3d_points(:,inlier)' - repmat(centroid_old, nnz(inlier), 1))' * (new_3d_points(:,inlier)' - repmat(centroid_new, nnz(inlier), 1));
[U,S,V] = svd(H);
Rot = V*U';
if det(Rot)<0
    V(:,3) = -V(:,3);
    Rot = V*U';
end
trn = - Rot*centroid_old' + centroid_new';

se3 = rt2tr(Rot,trn);