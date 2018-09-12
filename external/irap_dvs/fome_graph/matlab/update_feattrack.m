function feattrack = update_feattrack(feattrack,feat_live_idx,node)
[u, v, valids] = uvfromidx_undistort(feat_live_idx{node});
if node == 1
    for i=1:length(feat_live_idx{node})
        if valids(i)
            feattrack(i) = pointTrack(uint32(node),[u(i),v(i)]);
        else
            feattrack(i) = pointTrack(0,[0,0]);
        end
    end
else
    for i=1:length(u)
        if valids(i)
            if (feattrack(i).ViewIds == 0)
                feattrack(i) = pointTrack(uint32(node),[u(i),v(i)]);
            else
                feattrack(i).ViewIds(end+1) = node;
                feattrack(i).Points(end+1,:) = [u(i), v(i)];
            end
        end
    end
end