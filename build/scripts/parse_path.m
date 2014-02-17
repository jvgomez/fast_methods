function path = parse_path (filename)

fid = fopen(filename);
%celltype = fgetl(fid);
fgetl(fid);
data = fscanf(fid, '%f');

leafsize = data(1);
ndims = data(2);

dimsize = zeros(1,ndims);
for i = 1:ndims
    dimsize(i) = data(i+2);
end

path = data(i+3:end);

path = idx2coord(ndims, dimsize, path);

%grid = reshape(cells, dimsize);

% To see 2D slides, take into account that Matlab has another axes
% reference.
% imagesc(grid(:,:,1)')
% axis xy;
% axis image;

