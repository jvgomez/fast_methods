%% Dummy script to generate the random maps used in the benchmarking.

% Number of dimensions.
nd = 2;
% nd = 3;
% nd = 4;

% Number of total cells.
cells = 4e6;

side = round(cells^(1/nd));

for vmin = 0.0:0.2:0.8 % For each velocity ranges
    if nd == 2
        cells = side*side;
        % Savig up to 2 decimals.
        map = round((vmin + (1-vmin)*rand(side,side))*100)/100;
        saveGridToFile(map, ['maps/2drandom_' num2str(vmin) '.grid']);
    elseif nd == 3
        cells = side*side*side;
        % Savig up to 2 decimals.
        map = round((vmin + (1-vmin)*rand(side,side,side))*100)/100;
        saveGridToFile(map, ['maps/3drandom_' num2str(vmin) '.grid']);
    elseif nd == 4
        cells = side*side*side*side;
        % Savig up to 2 decimals.
        map = round((vmin + (1-vmin)*rand(side,side,side, side))*100)/100;
        saveGridToFile(map, ['maps/4drandom_' num2str(vmin) '.grid']);
    end
end