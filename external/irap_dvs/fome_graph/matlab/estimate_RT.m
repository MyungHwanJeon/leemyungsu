function [R, t] = estimate_RT(pt1,pt2,cameraParams)
K = cameraParams.IntrinsicMatrix;
% outlier rejection with MSAC
[~, inlier] = estimateEssentialMatrix(pt1(:,1:2),pt2(:,1:2),cameraParams,'Confidence',99,'MaxDistance',5);
ipt1 = pt1(inlier,:);
ipt2 = pt2(inlier,:);
inlier_ratio = nnz(inlier)/length(inlier);
fprintf("3d point inlier : %2.2f%%\n",100*inlier_ratio);

%invert depth to normalized pts
for o=1:length(ipt1)
    ipt1(o,:) = K\[ipt1(o,1:2) 1/ipt1(o,3)]';
end
for n=1:length(ipt2)
    ipt2(n,:) = K\[ipt2(n,1:2) 1/ipt2(n,3)]';
end

%se3 estimation with svd
se3_estimate_svd = guess_se3_svd(ipt1,ipt2);
[R,t] = tr2rt(se3_estimate_svd);
end

function se3 = guess_se3_svd(new_3d_points,old_3d_points)
newpts = new_3d_points';
oldpts = old_3d_points';

depthok = false(length(newpts(:,:)),1);
depthok_idx = [];
for l=1:1:length(depthok)
    if (~isnaninf(oldpts(:,l)) && ~isnaninf(newpts(:,l)))
        depthok(l) = true;
        depthok(l) = true;
        depthok_idx = [depthok_idx; l];
    end
end

if (nnz(depthok) < 8)
    se3 = rt2tr(eul2rotm([0 0 0]),[0 0 0]);
    return;
end

%find optimal value
centroid_old = [mean(oldpts(1,depthok_idx)) mean(oldpts(2,depthok_idx)) mean(oldpts(3,depthok_idx))];
centroid_new = [mean(newpts(1,depthok_idx)) mean(newpts(2,depthok_idx)) mean(newpts(3,depthok_idx))];
H = (oldpts(:,depthok)' - repmat(centroid_old, nnz(depthok), 1))' * (newpts(:,depthok)' - repmat(centroid_new, nnz(depthok), 1));
[U,~,V] = svd(H);
Rot = V*U';
if det(Rot)<0
    V(:,3) = -V(:,3);
    Rot = V*U';
end
trn = - Rot*centroid_old' + centroid_new';
se3 = rt2tr(Rot,trn);
end

function out = isnaninf(matrix)
out = false;
    for i = 1:length(matrix)
        if (isnan(matrix(i)) || isinf(matrix(i)))
            out = true;
        end
    end
end