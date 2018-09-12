%set constants
K = [199.092366542, 0.0, 132.192071378;
	0.0, 198.82882047, 110.712660011;
	0.0, 0.0, 1.0];
cameraParams = cameraParameters('IntrinsicMatrix', K);
dc = 100;
event_potential = zeros(240,180);
gaussian = (1/16)*[1 2 1; 2 4 2; 1 2 1];
dvst = 1;
figure(1)
imagesc(event_potential');
hold on

imgidx = 1;
while (sampledvs{1,dvst}.t < gts{1,gtt+100}.t)
    if (dvst >= length(sampledvs))
        break;
    end
    dt = sampledvs{1,dvst+1}.t - sampledvs{1,dvst}.t;
    x = sampledvs{1,dvst}.x + 1;
    y = sampledvs{1,dvst}.y + 1;
    p = 2*sampledvs{1,dvst}.p - 1;
    event_potential = exp(-dc*dt)*event_potential;
    event_potential(x-1:x+1,y-1:y+1) = event_potential(x-1:x+1,y-1:y+1) + p*gaussian;
%     event_potential(x,y) = event_potential(x,y) + p;
    if rem(dvst,5000)==0
        filename = ['/home/jhlee/data/' 'sample' sprintf('%06i',imgidx) '.png'];
        imwrite(event_potential',filename);
        imgidx = imgidx + 1;
        imagesc(event_potential');
        drawnow
    end
    dvst = dvst + 1;
end