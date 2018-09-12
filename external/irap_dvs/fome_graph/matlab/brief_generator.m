function pattern = brief_generator(BRIEF_n,window_size)
x1 = zeros(BRIEF_n,1);
x2 = zeros(BRIEF_n,1);
y1 = zeros(BRIEF_n,1);
y2 = zeros(BRIEF_n,1);

for i = 1:BRIEF_n
    x1(i) = floor(normrnd(0,0.04*window_size^2));
    while x1(i)>floor(window_size/2) || x1(i)<ceil(-window_size/2)
         x1(i) = floor(normrnd(0,0.04*window_size^2));
    end
    x2(i) = floor(normrnd(0,0.04*window_size^2));
    while x2(i)>floor(window_size/2) || x2(i)<ceil(-window_size/2)
         x2(i) = floor(normrnd(0,0.04*window_size^2));
    end
    y1(i) = floor(normrnd(0,0.04*window_size^2));
    while y1(i)>floor(window_size/2) || y1(i)<ceil(-window_size/2)
         y1(i) = floor(normrnd(0,0.04*window_size^2));
    end
    y2(i) = floor(normrnd(0,0.04*window_size^2));
    while y2(i)>floor(window_size/2) || y2(i)<ceil(-window_size/2)
         y2(i) = floor(normrnd(0,0.04*window_size^2));
    end
end   
pattern = [y1 x1 y2 x2];
end