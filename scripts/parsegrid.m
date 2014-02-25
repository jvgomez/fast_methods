clear all; close all;

fid = fopen('../debug/test_at.txt');
celltype = fgetl(fid);
data = fscanf(fid, '%f');

leafsize = data(1);
ndims = data(2);

dimsize = zeros(1,ndims);
for i = 1:ndims
    dimsize(i) = data(i+2);
end

cells = data(i+3:end);

grid = reshape(cells, dimsize);

%path = parse_path('test_path.txt');

%start_point = [101 ,294];
%path2 = compute_geodesic(grid,start_point); %Using Grabriel Peyre's Toolbox.


% To see 2D slides, take into account that Matlab has another axes
% reference.
imagesc(grid(:,:,1)')
axis xy;
axis image;

hold on
% plot(path(:,1),path(:,2),'w');
% plot(path2(1,:),path2(2,:), 'k.');
