function [feat_live_num,feat_live_idx] = find_alive_feats_2d(feat2ds,iter,ba_size)
%%find alive feature from <iter> to <iter+ba_size-1>
feat_live_idx = cell(ba_size,1);
front = iter;
back = iter+ba_size-1;
feat_live_num = [];
appearlist = [];

%find alive numbers along range
for i = front:back
    numcurr = feat2ds{i}.num;
    depcurr = feat2ds{i}.idepth;
    for j = 1:length(numcurr)
        if ~ismember(numcurr(j),appearlist)
            % if not seen before, update appearlist
            appearlist(end+1) = numcurr(j);
        else
            % if seen before, update numlist
            if (depcurr(j) ~= 0) && (~ismember(numcurr(j),feat_live_num))
                feat_live_num(end+1) = numcurr(j);
            end
        end
    end
end
feat_live_num = sort(feat_live_num);

%update liveidx upon livenum
for i = front:back
    numcurr = feat2ds{i}.num;
    idxcurr = feat2ds{i}.idx;
    liveidx = [];
    for j = 1:length(feat_live_num)
        if ismember(feat_live_num(j),numcurr)
            pos = (feat_live_num(j)==numcurr);
            liveidx(end+1) = idxcurr(pos);
        else
            liveidx(end+1) = 0;
        end
    end
    feat_live_idx{i-iter+1} = liveidx;
end

end