%set constants
dc = 100;
event_potential = zeros(346,260);

%read signal in time order
imut = 1;
dvst = 1;
fprintf('Synchronizing event and imu start time...')
if (events{1,dvst}.t < imus{1,imut}.t)
    while (events{1,dvst}.t < imus{1,imut}.t)
       dvst = dvst + 1;
    end
else
    while (events{1,dvst}.t > imus{1,imut}.t)
       imut = imut + 1;
    end
    while (events{1,dvst}.t < imus{1,imut}.t)
       dvst = dvst + 1;
    end
end
fprintf('Done\n')

poses = {};
for time=10:10:19800
    fprintf('time : %.4f\n',time/1000);
    next_t = imus{1,imut+time}.t;
    if (time~=10)
        prev_t = imus{1,imut+time-10}.t;
        event_potential = exp(-dc*(next_t-prev_t)) * event_potential; %decay
    end
    %update event potential for fixed time interval
    while (events{1,dvst}.t < next_t)
        dt = next_t - events{1,dvst}.t;
        x = events{1,dvst}.x+1;
        y = events{1,dvst}.y+1;
        p = 2*events{1,dvst}.p - 1;
        event_potential(x,y) = event_potential(x,y) + p*exp(-dc*dt); %update
        dvst = dvst + 1;
    end

    % cluster with activation-sorted segments (interval 10%)
%     nz_ = nnz(event_potential(:));
%     t04 = 89960-round(4*nz_/100):89960;
%     t08 = 89960-round(8*nz_/100):89960-round(4*nz_/100);
%     t12 = 89960-round(12*nz_/100):89960-round(8*nz_/100);
%     t16 = 89960-round(16*nz_/100):89960-round(12*nz_/100);
%     t20 = 89960-round(20*nz_/100):89960-round(16*nz_/100);
%     idxlist= {t04 t08 t12 t16 t20};
%     [~,idx] = sort(event_potential(:));

    % cluster with DBSCAN (top 5% activation)
    cidx=dbscan_ev(idx(89960:-1:89960*0.95),event_potential,10,4);

    % compute optical flow with LK method
    flow_x = zeros(346,260);
    flow_y = zeros(346,260);
    idepth = zeros(346,260);
    [diff_x,diff_y] = gradient(event_potential);
    diff_t = -(1/dc)*event_potential;
    for pt=cidx
        A = [diff_x(cat(1,pt{:})); diff_y(cat(1,pt{:}))];
        b = diff_t(cat(1,pt{:}));
        uv = (A*A')\A*b';
        flow_x(idx(cat(1,pt{:}))) = uv(1);
        flow_y(idx(cat(1,pt{:}))) = uv(2);
        idepth(idx(cat(1,pt{:}))) = norm(uv);
    end

    % compute optical flow with Gradient sum
%     flow_x = zeros(346,260);
%     flow_y = zeros(346,260);
%     idepth = zeros(346,260);
%     [diff_x,diff_y] = gradient(event_potential);
%     diff_t = -(1/dc)*event_potential;
%     for clusterpts=idxlist
%         for pts = idx(cat(1,clusterpts{:}))'
%             location_x = round(pts/260)+1;
%             location_y = rem(pts,260)+1;
%             if ((location_x == 1) || (location_x == 346) || (location_y == 1) || (location_y == 346))
%                 continue;
%             end
%             flow_x(idx(pts)) = diff_x(location_x-1)-diff_x(location_x+1);
%             flow_y(idx(pts)) = diff_y(location_y+1)-diff_y(location_y-1);
%             idepth(idx(pts)) = norm([flow_x(idx(pts)), flow_y(idx(pts))]);
%         end
%     end
    
    % compute optical flow with Gradient Descent

    %compute 3d point transformation with optical flow info
    point_3d_tf = zeros(3,260,346);
    point_3d_or = zeros(3,260,346);
    for i=1:1:346
        for j = 1:1:260
            if (idepth(i,j) ~= 0)
                point_3d_or(:,j,i) = (1/idepth(i,j))*K\[undistort_map_x(j,i); undistort_map_y(j,i); 1];
                tf_2d_point = K*(point_3d_or(:,j,i)) + [flow_x(i,j); flow_y(i,j); 0];
                point_3d_tf(:,j,i) = (1/idepth(i,j))*K\tf_2d_point;
            end
        end
    end
    se3_estimate_ = make_guess_se3_svd(point_3d_tf,point_3d_or);
    [R_,t_] = tr2rt(se3_estimate_);
    poses{time/10}.rot = R_;
    poses{time/10}.trn = t_;
    poses{time/10}.time = imus{1,imut+time}.t;
end

%plot the result
figure(1)
hold on
initial_pose = [0 0 0 0 0 0]';
integrated_pose = zeros(length(poses)+1,6);
for a=1:length(poses)
    scatter3(integrated_pose(a,1),integrated_pose(a,2),integrated_pose(a,3),1);
    integrated_pose(a+1,1:3) = integrated_pose(a,1:3)+(eul2rotm(integrated_pose(a,4:6))*poses{a}.trn)';
    integrated_pose(a+1,4:6) = integrated_pose(a,4:6)+rotm2eul(poses{a}.rot);
end

%plot the result
figure(2)
hold on
gti = 1;
while (gts{1,gti}.t < imus{1,1}.t)
	gti = gti + 1;
end
for gtt=gti:length(gts)
    scatter3(gts{1,gtt}.x,gts{1,gtt}.y,gts{1,gtt}.z,1);
end


% %compute gt diffrence with timestamp
% gtt = 1;
% while (gts{1,gtt}.t < imus{1,imut}.t)
% 	gtt = gtt + 1;
% end
% gt_dt = gts{1,gtt}.t - gts{1,gtt-1}.t;
% gt_eul = quat2eul([gts{1,gtt}.t_x gts{1,gtt}.t_y gts{1,gtt}.t_z gts{1,gtt}.t_w])...
%         - quat2eul([gts{1,gtt-1}.t_x gts{1,gtt-1}.t_y gts{1,gtt-1}.t_z gts{1,gtt-1}.t_w]);
% gt_xyz = [gts{1,gtt}.x gts{1,gtt}.y gts{1,gtt}.z] - [gts{1,gtt-1}.x gts{1,gtt-1}.y gts{1,gtt-1}.z];
% 
% original_cam_pt = [1000; 5000; 1500];
% tf_gt = eul2rotm(gt_eul)*original_cam_pt + gt_xyz';
% tf_est = R_*original_cam_pt + t_;
% disp([original_cam_pt, tf_gt, tf_est])
% fprintf('gt dist : %.3f\n', norm(tf_gt-original_cam_pt));
% fprintf('estimate dist : %.3f\n', norm(tf_gt-tf_est));