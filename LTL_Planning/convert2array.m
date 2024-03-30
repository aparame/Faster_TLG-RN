function X = convert2array(locations, gridW)
X = zeros([1,height(locations)]);
for i = 1:height(locations)
    X(i) = (locations(i,2)-1)*gridW + locations(i,1);
end