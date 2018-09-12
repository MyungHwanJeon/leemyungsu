K = [226.38018519795807 0 173.6470807871759;
    0 226.15002947047415 133.73271487507847;
    0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix', K);

R_orig = [0.06 -0.05 0.02];
t_orig = [0.50 0.30 -0.10]';
depth = linspace(0.6,0.8,260)'*linspace(0.4,0.7,346);

point_3d_tf = zeros(3,260,346);
point_3d_or = zeros(3,260,346);

fprintf('generating noisy data...')
for i=1:1:346
    for j = 1:1:260
        R_noisy = R_orig + 0.03*randn(1,3);
        t_noisy = t_orig + 0.1*randn(1,3);
        newpt = eul2rotm(R_noisy)*(depth(j,i)*K\[undistort_map_x(j,i); undistort_map_y(j,i); 1]) + t_noisy;
        point_3d_tf(:,j,i) = newpt(1:3);
        point_3d_or(:,j,i) = depth(j,i)*K\[undistort_map_x(j,i); undistort_map_y(j,i); 1];
    end
end
fprintf('done\n')

se3_estimate_svd = make_guess_se3_svd(point_3d_tf,point_3d_or);
se3_estimate_fmin = make_guess_se3_fmin(point_3d_tf,point_3d_or);
[R_svd,t_svd] = tr2rt(se3_estimate_svd);
[R_min,t_min] = tr2rt(se3_estimate_fmin);

original_cam_pt = [435; 547; 346];
tf_gt = eul2rotm(R_orig)*original_cam_pt + t_orig;
tf_svd = R_svd*original_cam_pt + t_svd;
tf_min = R_min*original_cam_pt + t_min;
disp('svd euclidean dist :')
disp(norm(tf_gt-tf_svd))
disp('fmin euclidean dist :')
disp(norm(tf_gt-tf_min))