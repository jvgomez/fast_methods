%% Dummy script to generate the random maps used in the benchmarking.

% Number of dimensions.
nd = 2;
% nd = 3;
% nd = 4;

% Number of total cells.
cells = 4e6;

side = round(cells^(1/nd));
side10 = round(side/10);
for vmin = 0.0:0.2:0.8 % For each velocity ranges
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
            map(map==0) = 0.01;
        else
            map(map==0) = vmin;
        end
        saveGridToFile(map, ['maps/2dchecker_' num2str(vmin) '.grid']);
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
            map(map==0) = 0.01;
        else
            map(map==0) = vmin;
        end
        saveGridToFile(map, ['maps/3dchecker_' num2str(vmin) '.grid']);
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
            map(map==0) = 0.01;
        else
            map(map==0) = vmin;
        end
        saveGridToFile(map, ['maps/4dchecker_' num2str(vmin) '.grid']);
    end
end