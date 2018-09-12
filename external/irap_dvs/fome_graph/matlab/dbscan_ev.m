function clusters=dbscan_ev(iidx,event_potential,threshold,minpts)
%%naive dbscan algorithm (search for every point, slow)
num = size(iidx,1);
groupnum = zeros(num,1);
Dist = squareform(pdist([round(iidx/346) mod(iidx,346) 100*event_potential(iidx)]));
clusters = {};
visited = false(num,1);

for i=1:num
    if visited(i)
        continue;
    end
    visited(i) = true;
    nbdpts = ngbd(i);
    if numel(nbdpts) < minpts
        groupnum(i) = -1; %is noise and not assigned for any cluster.
    else
        if (groupnum(i) == 0) % if not assigned for cluster, set as new cluster
            groupnum(i) = length(clusters);
        end
        cluster = ExpandCluster(iidx(i),nbdpts,groupnum(i));
        clusters{length(clusters)+1} = cluster;
    end
end

    function cluster = ExpandCluster(point,nbd,groupno)
        groupnum(point) = groupno;
        n_th_nbd = 1;
        while true
            nbd_now = nbd(n_th_nbd);
            if ~visited(nbd_now)
                visited(nbd_now) = true;
                nbd_of_nbd = ngbd(nbd_now);

                if numel(nbd_of_nbd) < minpts
                    groupnum(nbd_now) = -1; %is noise
                else
                    groupnum(nbd_now) = groupno;
                    for it=1:length(nbd)
                        nbd_of_nbd(nbd_of_nbd == nbd(it)) = [];
                    end
                    nbd = [nbd nbd_of_nbd];
                end
            end
            n_th_nbd = n_th_nbd + 1;
            if n_th_nbd > numel(nbd)
                break;
            end
        end
        cluster = iidx(nbd);
    end

    function closepts = ngbd(i)
        closepts = find(Dist(i,:)<=threshold);
    end

end