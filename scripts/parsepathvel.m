function [path, vel] = parsepathvel (filename)

fid = fopen(filename);
data = fscanf(fid, '%f');
fclose(fid);

leafsize = data(1);
ndims = data(2);

dimsize = zeros(1,ndims);
for i = 1:ndims
    dimsize(i) = data(i+2);
end

path = zeros((size(data,1)-(ndims+2))/ndims , ndims);
count = 0;
for i = ndims+3:ndims+1:size(data,1)
    count = count + 1;
    for j = 1:ndims
       path(count,j) = data(i+j-1) +1; % The +1 adapts the array indexing from C++ to Matlab.
    end
    vel(count) = data(i+j);
end
