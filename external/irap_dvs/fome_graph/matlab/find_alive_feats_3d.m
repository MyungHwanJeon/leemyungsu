function [xyz, num, idx] = find_alive_feats_3d(feat3ds,iter,ba_size,feat_live_num, feat_live_idx)
%%find alive feature from <iter> to <iter+ba_size-1>
xyz = zeros(length(feat_live_num),3);
front = iter;
back = iter+ba_size-1;
not_found = true(size(feat_live_num));
idx = {};
%copy xyz positions from feat3ds, from back to front
for now = linspace(back,front,back-front+1)
    numlist = feat3ds{now}.id;
    xyzlist = feat3ds{now}.xyz;
    for i = 1:length(numlist)
        %%if nan or inf, just pass
        if ismember(numlist(i),feat_live_num) % if exists in live_num, try register.
            foundnum = (feat_live_num==numlist(i));
            if not_found(foundnum) %if found for the first time, register.
                xyz(foundnum,:) = xyzlist(i,:);
                not_found(foundnum) = false;
            else
                %if already found, do nothing
            end
        else
            %if not exists in live_num, do nothing.
        end
    end
end
if nnz(not_found==true) > 0 %if something is not found, erase that feat number.
    num = feat_live_num(~not_found);
    for i=1:length(feat_live_idx)
        liveidx = feat_live_idx{i};
        idx{i} = liveidx(~not_found);
    end
    xyz = xyz(~not_found,:);
else
    idx = feat_live_idx;
    num = feat_live_num;
end

end