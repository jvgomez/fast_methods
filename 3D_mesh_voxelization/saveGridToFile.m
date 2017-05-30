%% Save a nDimensional grid to a text file which format is ready to
%% be read by the FM code.

function saveGridToFile(grid, filename, leafsize)
    fileID = fopen(filename, 'w');
    
    fprintf(fileID, '%s\n', 'FMCell - Fast Marching Cell');
    if nargin > 2
        fprintf(fileID, '%f\n', leafsize);
    else
        fprintf(fileID, '1.0\n');
    end
    fprintf(fileID, '%d\n', ndims(grid));
    
    for i = 1:ndims(grid)
         fprintf(fileID, '%d\n', size(grid,i));
    end
    
    cells = grid(:);
    fprintf(fileID, '%.2f\n', cells);
  
    
    fclose(fileID);