function feat_3d = update_3d_features(feat_2d,feat_3d,state_new,cameraParams)
K = cameraParams.IntrinsicMatrix;
iter = 1;
for i=1:length(feat_3d.id)
    if ~ismember(feat_3d.id(i),feat_2d.num)
        feat_3d.id(i) = 0;
    end
end
feat_3d.id = feat_3d.id(feat_3d.id~=0);
feat_3d.xyz = feat_3d.xyz(feat_3d.id~=0,:);
feat_3d.cov = feat_3d.cov(feat_3d.id~=0);

Rot = eul2rotm(state_new(1:3));

for j=1:length(feat_2d.num)
    if feat_2d.idepth(j)==0
        continue;
    end
    [u, v] = uvfromidx_undistort(feat_2d.idx(j));
    d = 1/feat_2d.idepth(j);
    covmat = [1 0 0; 0 1 0; 0 0 1e6*feat_2d.cov(j)*(d^2)];
    xyz_new = state_new(4:6)' + Rot*d*(K\[u;v;1]);
    cov_new = invdepth_to_cartesian_cov(covmat,K\[u;v;1],feat_2d.idepth(j));
    
    if ~ismember(feat_2d.num(j),feat_3d.id)
        %register newly added features
        new_3d.xyz(iter,:) = xyz_new;
        new_3d.id(iter) = feat_2d.num(j);
        new_3d.cov(iter) = {cov_new};
        iter = iter + 1;
    else
        %update observed 3d features
        curr = find(feat_3d.id==feat_2d.num(j));
        cov_old = feat_3d.cov{curr};
        xyz_old = feat_3d.xyz(curr,:);
        
        cov = inv(inv(cov_old)+inv(cov_new));
        xyz = (inv(cov_old)+inv(cov_new))\(cov_old\xyz_old' + cov_new\xyz_new);
        feat_3d.cov{curr} = cov;
        feat_3d.xyz(curr,:) = xyz;
    end
end
if (iter ~= 1)
    feat_3d.xyz = [feat_3d.xyz; new_3d.xyz];
    feat_3d.id = [feat_3d.id, new_3d.id];
    feat_3d.cov = [feat_3d.cov, new_3d.cov];
end

end

function covout = invdepth_to_cartesian_cov(cov,point,rho)
    theta = atan2(point(1),1);
    phi = atan2(point(2),1);
    mi = m(theta,phi);
    dm_dtheta = [cos(phi)*cos(theta)  0   -cos(phi)*sin(theta)]';
    dm_dphi = [-sin(phi)*sin(theta)  -cos(phi)   -sin(phi)*cos(theta)]';
    J = [ (1/rho)*dm_dtheta (1/rho)*dm_dphi -mi/(rho^2) ];
    covout = J*cov*J';
end

function m = m(a,b)
% Unit vector from azimut-elevation angles
    theta = a; phi = b;
    cphi = cos(phi);
    m = [cphi.*sin(theta)   -sin(phi)  cphi.*cos(theta)]';
end