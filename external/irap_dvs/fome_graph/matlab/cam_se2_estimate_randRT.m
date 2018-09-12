K = [226.38018519795807 0 173.6470807871759;
    0 226.15002947047415 133.73271487507847;
    0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix', K);

R_orig = [0.06 -0.05 0.02];
t_orig = [0.50 0.30 -0.10]';
depth = linspace(0.6,0.8,260)'*linspace(0.4,0.7,346);

point_2d_tf = zeros(2,260,346);
point_2d_or = zeros(2,260,346);

fprintf('generating noisy data...')
for i=1:1:346
    for j = 1:1:260
%         R_noisy = R_orig + 0.03*randn(1,3);
%         t_noisy = t_orig + 0.1*randn(3,1);
        R_noisy = R_orig + 0.00*randn(1,3);
        t_noisy = t_orig + 0.0*randn(3,1);
        new_3d_pt = eul2rotm(R_noisy)*(depth(j,i)*K\[undistort_map_x(j,i); undistort_map_y(j,i); 1]) + t_noisy;
        new_2d_pt = K*(new_3d_pt/new_3d_pt(3));
        point_2d_tf(:,j,i) = new_2d_pt(1:2);
        point_2d_or(:,j,i) = [undistort_map_x(j,i); undistort_map_y(j,i)];
    end
end
fprintf('done\n')

%randomly generate inliers
inlier = false(1,260*346);
for i=1:length(inlier)
    if rand()>0.98
        inlier(i) = true;
    end
end
pt_or = point_2d_or(:,inlier);
pt_tf = point_2d_tf(:,inlier);
[R,t] = estimate_RT(pt_or',pt_tf',cameraParams);
tx = atan2(t(3),t(1));
ty = atan2(t(2),t(3));
gtx = atan2(t_orig(3),t_orig(1));
gty = atan2(t_orig(3),t_orig(2));

fprintf('Rot = %.4f %.4f\n',rotm2eul(R),R_orig);
fprintf('trn = %.4f %.4f\n',[tx gtx],[ty gty]);