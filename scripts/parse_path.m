function path = parse_path (filename)

fid = fopen(filename);
data = fscanf(fid, '%f');
fclose(fid);

leafsize = data(1);
ndims = data(2);

dimsize = zeros(1,ndims);
for i = 1:ndims
    dimsize(i) = data(i+2);
end

path = [data(ndims+3:2:end) data(ndims+4:2:end)];
%path = path(1:10:end,:);