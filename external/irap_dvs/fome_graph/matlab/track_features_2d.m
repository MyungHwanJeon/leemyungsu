function features = track_features_2d(features, event_potential,pattern,BRIEF_n)
rows = 346;
cols = 260;
im_corner =zeros(floor(15/2));
edge_ver = zeros(rows,floor(15/2));
edge_hor = zeros(floor(15/2),cols);
test = [im_corner edge_hor im_corner;...
        edge_ver event_potential edge_ver;...
        im_corner edge_hor im_corner;];
y1 = pattern(:,1);
x1 = pattern(:,2);
y2 = pattern(:,3);
x2 = pattern(:,4);

    for i=1:length(features.idx)
        x = rem(features.idx(i)-1,346)+1;
        y = floor((features.idx(i)-1)/346)+1;
        if ((2>=x || 344<=x) || (2>=y || 258<=y))
            features.idx(i) = 0;
            continue;
        end
        now_patch = event_potential(x-2:x+2,y-2:y+2);
        % event descriptor matching
        [int_row,int_col] = find(abs(now_patch) >= abs(event_potential(x,y)));
        if isempty(int_row)
            if norm(now_patch) < 0.1
                features.idx(i) = 0;
            end
            continue;
        end
        int_x = x+int_row-3;
        int_y = y+int_col-3;
        ref_desc = features.desc(i,:);
        distance = zeros(1,length(int_x));
        for intpt = 1:length(int_x)
            int_desc = zeros(1,BRIEF_n);
            coord_x = int_x(intpt) + 7;
            coord_y = int_y(intpt) + 7;
            for j = 1:BRIEF_n
                if test(coord_x+x1(j),coord_y+y1(j))<test(coord_x+x2(j),coord_y+y2(j))
                    int_desc(j) = true;
                else
                    int_desc(j) = false;
                end
            end
            
%             int_patch_relative = event_potential(px-2:px+2,py-2:py+2) - event_potential(px,py);
%             level_diff = abs(ref_patch(3,3) - event_potential(px,py)); %% if score same, goto the more similar evlv.
            distance(intpt) = sum(xor(ref_desc,int_desc));
%             scores(intpt) = norm(ref_patch_binary - int_patch_binary) - dist;
%             scores(intpt) = norm(ref_patch_relative - int_patch_relative) - dist;
        end
        [~,minidx] = min(distance);
        x_new = int_x(minidx);
        y_new = int_y(minidx);
        
%         % iterative calm-region-crawl approach
%         [calm,pos] = find_calmest(now_patch);
% 
%         largerpatch = event_potential(x-4:x+4,y-4:y+4);
%         shapefactor = 100*ones(5);
%         rp_n = ref_patch;
%         for xp=pos(1,:)
%             for yp=pos(2,:)
%                 if ~calm(xp+3,yp+3)
%                     continue;
%                 end
%                 np_n = largerpatch(xp+3:xp+7,yp+3:yp+7);
%                 shapefactor(xp+3,yp+3) = norm(np_n-rp_n); %lower shapefactor is better
%             end
%         end
% 
% %         valuefactor = abs(now_patch)-abs(ref_patch); %larger valuefactor is better
% %         [~,Idxcol] = max(max(3*valuefactor-shapefactor));
% %         [~,Idxrow] = max(max((3*valuefactor-shapefactor)'));
%         [~,Idxcol] = min(min(shapefactor));
%         [~,Idxrow] = min(min((shapefactor)'));
%         x_new = x+Idxrow-3;
%         y_new = y+Idxcol-3;
        if ((7>=x_new || 340<=x_new) || (7>=y_new || 254<=y_new))
            features.idx(i) = 0;
            continue;
        else
            features.idx(i) = x_new+346*(y_new-1);
%             features.patch(i) = {now_patch};
        end
    end
    if nnz(features.idx==0)~=0
%         fprintf('removing %i feature(s) by patch ambiguity\n', nnz(features.idx==0));
        features.num = features.num(features.idx~=0);
        features.idx = features.idx(features.idx~=0);
        features.desc = features.desc(features.idx~=0,:);
        features.idepth = features.idepth(features.idx~=0,:);
    end
end