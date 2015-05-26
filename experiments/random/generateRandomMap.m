%% Dummy script to generate the random maps used in the benchmarking.

%% NOTE: files called checker for simplicity!!
% Number of dimensions.
% nd = 2;
% nd = 3;
nd = 4;

% Number of total cells.
cells = 4e6;

mkdir('maps');
side = round(cells^(1/nd));
for vmin = 0:10:100 % For each velocity ranges
    vmin2 = max(1,vmin);
    if nd == 2
        cells = side*side;
        % Savig up to 2 decimals.
        map = round((vmin2 + (1-vmin2)*rand(side,side))*100)/100;
        saveGridToFile(map, ['maps/2drandom_' num2str(vmin2) '.grid']);
    elseif nd == 3
        cells = side*side*side;
        % Savig up to 2 decimals.
        map = round((vmin2 + (1-vmin2)*rand(side,side,side))*100)/100;
        saveGridToFile(map, ['maps/3drandom_' num2str(vmin2) '.grid']);
    elseif nd == 4
        cells = side*side*side*side;
        % Savig up to 2 decimals.
        map = round((vmin2 + (1-vmin2)*rand(side,side,side,side))*100)/100;
        saveGridToFile(map, ['maps/4drandom_' num2str(vmin2) '.grid']);
    end
end

% Command to save 2d to pdf: 
% image(map./100*255); axis image; box on; set(gca,'XTick',[]);set(gca,'YTick',[]); colormap gray(256);
