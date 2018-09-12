function features = add_features(features,event_potential,pattern,lownum,BRIEF_n)
new_features_idx = [];
new_features_num = [];
new_features_desc = [];
new_features_idepth = [];
[~,idx] = sort(event_potential(:));
pospt = 89960;
negpt = 1;
numcounter = max(features.num)+1;
if isempty(numcounter)
    numcounter = 1;
end

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

while size(new_features_idx) < 1.2*lownum-length(features.idx)
    if round(rand()) %% randomly select between neg/pos features
        if isempty(find(features.idx==idx(pospt),1))
            [x, y] = uvfromidx(idx(pospt),346);
%             if ((7>=x || 340<=x) || (7>=y || 254<=y))
%                 pospt = pospt - ceil(3*rand());
%                 continue;
%             end
            descriptor = false(1,BRIEF_n);
            coord_x = x + 7;
            coord_y = y + 7;
            for desc = 1:BRIEF_n
                if test(coord_x+x1(desc),coord_y+y1(desc))<test(coord_x+x2(desc),coord_y+y2(desc))
                    descriptor(desc) = true;
                else
                    descriptor(desc) = false;
                end
            end
            
            numcounter = numcounter + 1;
            new_features_num = [new_features_num numcounter];
            new_features_idx = [new_features_idx idx(pospt)];
            new_features_desc = [new_features_desc; descriptor];
            new_features_idepth = [new_features_idepth; 0];
        end
%         pospt = pospt - 1;
        pospt = pospt - ceil(5*rand());
    else
        if isempty(find(features.idx==idx(negpt),1))
            [x, y] = uvfromidx(idx(pospt),346);
%             if ((7>=x || 340<=x) || (7>=y || 254<=y))
%                 negpt = negpt + ceil(3*rand());
%                 continue;
%             end
            descriptor = false(1,BRIEF_n);
            coord_x = x + 7;
            coord_y = y + 7;
            for desc = 1:BRIEF_n
                if test(coord_x+x1(desc),coord_y+y1(desc))<test(coord_x+x2(desc),coord_y+y2(desc))
                    descriptor(desc) = true;
                else
                    descriptor(desc) = false;
                end
            end
            
            numcounter = numcounter + 1;
            new_features_num = [new_features_num numcounter];
            new_features_idx = [new_features_idx idx(negpt)];
            new_features_desc = [new_features_desc; descriptor];
            new_features_idepth = [new_features_idepth; 0];
        end
%         negpt = negpt + 1;
        negpt = negpt + ceil(5*rand());
    end
end
features.idx = [features.idx new_features_idx];
features.num = [features.num new_features_num];
features.cov = [features.cov zeros(size(new_features_idx))];
features.desc = [features.desc; new_features_desc];
features.idepth = [features.idepth; new_features_idepth];

end