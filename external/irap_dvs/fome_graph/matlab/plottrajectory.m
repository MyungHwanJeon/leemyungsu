function plottrajectory(featlist)
    for i=1:length(featlist)-1
        prevnum = featlist{i}.num;
        currnum = featlist{i+1}.num;
        [~, match2] = ismember(prevnum,currnum);
        [~, match1] = ismember(currnum,prevnum);
        previdx = featlist{i}.idx;
        curridx = featlist{i+1}.idx;
        order_p = previdx(match1(match1~=0));
        order_c = curridx(match2(match2~=0));
        
        xp = rem(order_p-1,346)+1;
        yp = floor((order_p-1)/346)+1;
        xc = rem(order_c-1,346)+1;
        yc = floor((order_c-1)/346)+1;
        for j =1:length(order_p)
            hd = plot([xp(j) xc(j)],[yp(j) yc(j)],'Color',[1-0.01*i 1-0.01*i 1-0.01*i]);
            set(hd,'linewidth',3);
        end
        drawnow
    end
end