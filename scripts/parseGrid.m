function grid = parseGrid(filename)

    fid = fopen(filename);
    celltype = fgetl(fid);
    data = fscanf(fid, '%f');

    leafsize = data(1);
    ndims = data(2);

    dimsize = zeros(1,ndims);
    for i = 1:ndims
        dimsize(i) = data(i+2);
    end

    cells = data(i+3:end);
    
    grid.celltype = celltype;
    grid.leafsize = leafsize;
    grid.ndims = ndims;
    grid.dimsize = dimsize;
    if ndims == 2
    	grid.cells = reshape(cells, dimsize)';
    else
        grid.cells = reshape(cells, dimsize);
    end