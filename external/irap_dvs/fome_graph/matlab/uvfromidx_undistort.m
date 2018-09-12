function [u, v, valids] = uvfromidx_undistort(idx)
global undistort_map_x undistort_map_y
valids = (idx~=0);
invalids = (idx==0);

u = undistort_map_x(idx(valids));
v = undistort_map_y(idx(valids));
for i =1:length(idx)
    if invalids(i)
        u = [u(1:i-1) 0 u(i:end)];
        v = [v(1:i-1) 0 v(i:end)];
    end
end
end