function [calmpatch,pos] = find_calmest(patch)
[row, col] = size(patch);
calmpatch = false(row,col);
patchx = (row+1)/2;
patchy = (col+1)/2;
% [ 1 2 ] 
% [ 3 4 ]
p1 = patch(1:patchx,1:patchy);
p2 = patch(1:patchx,col-patchy+1:col);
p3 = patch(row-patchx+1:row,1:patchy);
p4 = patch(row-patchx+1:row,col-patchy+1:col);

[~,calm] = min([norm(p1) norm(p2) norm(p3) norm(p4)]);
if calm == 1
    calmpatch(1:patchx,1:patchy) = true;
    posx = [-2 -2 -2 -1 -1 -1  0  0  0];
    posy = [-2 -1  0 -2 -1  0 -2 -1  0];
end
if calm == 2
    calmpatch(row-patchx+1:row,1:patchy) = true;
    posx = [ 0  0  0  1  1  1  2  2  2];
    posy = [-2 -1  0 -2 -1  0 -2 -1  0];
end
if calm == 3
    calmpatch(1:patchx,col-patchy+1:col) = true;
    posx = [-2 -2 -2 -1 -1 -1  0  0  0];
    posy = [ 0  1  2  0  1  2  0  1  2];
end
if calm == 4
    calmpatch(row-patchx+1:row,col-patchy+1:col) = true;
    posx = [ 0  0  0  1  1  1  2  2  2];
    posy = [ 0  1  2  0  1  2  0  1  2];
end
pos = [posx;posy];