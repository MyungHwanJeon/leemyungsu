K = [226.38018519795807 0 173.6470807871759;
    0 226.15002947047415 133.73271487507847;
    0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix', K);

R_ = [0.06 -0.05 0.02];
t_ = [0.05 0.03 -0.01]';
idepth = linspace(0.4,0.8,260)'*linspace(0.7,0.6,346);

point_3d_tf = zeros(3,260,346);
point_3d_or = zeros(3,260,346);

R_mea = [0.06 -0.05 0.02] + [0.02*randn() 0.02*randn() 0.02*randn()];

fprintf('generating noisy data...')
for i=1:1:346
    for j = 1:1:260
        point_3d_or(:,j,i) = (1/idepth(j,i))*K\[undistort_map_x(j,i); undistort_map_y(j,i); 1];
        tf_3d_point = (eul2rotm(R_)*point_3d_or(:,j,i) + t_);
        tf_2d_point = K*(tf_3d_point/tf_3d_point(3)) + [100*randn(); 100*randn(); 0.01*randn()];
        point_3d_tf(:,j,i) = (1/idepth(j,i))*K\tf_2d_point; %even after transformation, assume same depth.
    end
end
fprintf('done\n')

se3_estimate_svd = make_guess_se3_svd(point_3d_tf,point_3d_or);
se3_estimate_fmin = make_guess_se3_fmin(point_3d_tf,point_3d_or,[0 0 0]);
se3_estimate_fmin_imu = make_guess_se3_fmin(point_3d_tf,point_3d_or,R_mea);
[R_svd,t_svd] = tr2rt(se3_estimate_svd);
[R_min,t_min] = tr2rt(se3_estimate_fmin);
[R_imu,t_imu] = tr2rt(se3_estimate_fmin_imu);

original_cam_pt = [43; 54; 34];
tf_gt = eul2rotm(R_)*original_cam_pt + t_;
tf_svd = R_svd*original_cam_pt + t_svd;
tf_min = R_min*original_cam_pt + t_min;
tf_imu = R_imu*original_cam_pt + t_imu;
fprintf('svd euclidean dist : %.3f\n',norm(tf_gt-tf_svd));
fprintf('fmin euclidean dist : %.3f\n', norm(tf_gt-tf_min));
fprintf('fimu euclidean dist : %.3f\n', norm(tf_gt-tf_imu));
disp([tf_gt, tf_svd, tf_min, tf_imu])