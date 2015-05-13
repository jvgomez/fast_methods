%% Dummy script to generate the random maps used in the benchmarking.

% Number of dimensions.
nd = 2;
% nd = 3;
% nd = 4;

% Number of total cells.
cells = 4e6;

mkdir('maps');
side = round(cells^(1/nd));
side10 = round(side/10);
for vmin = 0:10:100 % For each velocity ranges
% for vmin = 100
    if nd == 2
        cells = side*side;
        map = ones(side, side);
        for i = side10:2*side10:side
            m = min(i+side10, side);
            % Rows
            map(i+1:m, :) = ~map(i+1:m, :);
            % Cols
            map(:, i+1:m) = ~map(:, i+1:m);
        end
        if vmin == 0
            map(map==0) = 1;
            saveGridToFile(map, ['maps/2dchecker_' num2str(1) '.grid']);
        else
            map(map==0) = vmin;
            saveGridToFile(map, ['maps/2dchecker_' num2str(vmin) '.grid']);
        end
    elseif nd == 3
        cells = side*side*side;
        map = ones(side, side, side);
        for i = side10:2*side10:side
            m = min(i+side10, side);
            % Rows
            map(i+1:m, :, :) = ~map(i+1:m, :, :);
            % Cols
            map(:, i+1:m, :) = ~map(:, i+1:m, :);
            % Heights
            map(:, :, i+1:m) = ~map(:, :, i+1:m);
        end
        if vmin == 0
            map(map==0) = 1;
            saveGridToFile(map, ['maps/3dchecker_' num2str(1) '.grid']);
        else
            map(map==0) = vmin;
            saveGridToFile(map, ['maps/3dchecker_' num2str(vmin) '.grid']);
        end
    elseif nd == 4
        cells = side*side*side*side;
        map = ones(side, side, side, side);
        for i = side10:2*side10:side
            m = min(i+side10, side);
            % Rows
            map(i+1:m, :, :, :) = ~map(i+1:m, :, :, :);
            % Cols
            map(:, i+1:m, :, :) = ~map(:, i+1:m, :, :);
            % Heights
            map(:, :, i+1:m, :) = ~map(:, :, i+1:m, :);
            % ehm...
            map(:, :, :, i+1:m) = ~map(:, :, :, i+1:m);
        end
        if vmin == 0
            map(map==0) = 1;
            saveGridToFile(map, ['maps/4dchecker_' num2str(1) '.grid']);
        else
            map(map==0) = vmin;
            saveGridToFile(map, ['maps/4dchecker_' num2str(vmin) '.grid']);
        end
    end
end

% Command for save 2d as pdf:
% image(map./100*255); axis image; box on; set(gca,'XTick',[]);set(gca,'YTick',[]); colormap gray(256);
