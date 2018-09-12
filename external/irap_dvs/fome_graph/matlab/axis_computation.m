function pcase = axis_computation(patch)
p1 = [-1 -1 -1; 1 1 1; -1 -1 -1];
p2 = [-1 -1 1; -1 1 -1; 1 -1 -1];
p3 = [-1 1 -1; -1 1 -1; -1 1 -1];
p4 = [1 -1 -1; -1 1 -1; -1 -1 1];

cout1 = norm(conv2(p1,patch));
cout2 = norm(conv2(p2,patch));
cout3 = norm(conv2(p3,patch));
cout4 = norm(conv2(p4,patch));

[~,pcase] = max([cout1 cout2 cout3 cout4]);

end