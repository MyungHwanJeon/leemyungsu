function se3 = make_guess_se3_fmin(new_3d_points,old_3d_points,R_imu)
if (nargin == 2)
    R_imu = [0 0 0];
end

inlier = false(length(new_3d_points(:,:)),1);
inlieridx = [];
for l=1:1:length(inlier)
     if (new_3d_points(3,l) ~= 0)
        inlier(l) = true;
        inlieridx = [inlieridx; l];
     end
end

%find optimal value
fprintf('finding R,t value...')
trn_init = [0;0;0];
for i=1:1:nnz(inlier)
    trn_init = trn_init + old_3d_points(:,inlieridx(i)) - new_3d_points(:,inlieridx(i));
end
trn_init = -trn_init / nnz(inlier);
func = @(x) norm(repmat(x(4:6),1,nnz(inlier)) + eul2rotm(x(1:3)')*old_3d_points(:,inlier) - new_3d_points(:,inlier));
x0 = [R_imu';trn_init];
rottrn = fminunc(func,x0);
Rot = eul2rotm(rottrn(1:3)');
trn = rottrn(4:6);

fprintf('done\n')

se3 = rt2tr(Rot,trn);