%% Coordinates given in MATLAB mode (1 to n)!!

function coords = idx2coord(ndims, dimsize, path)

d = dimsize(1);

for i = 2:ndims
    d(i) = d(i-1)*dimsize(i);
end

for j = 1:size(path,1)
    idx = path(j);
    coords(j,ndims) = floor(idx/d(ndims-1));
    aux = idx - coords(j,ndims)*d(ndims-1);
    for i = ndims-1:-1:2
        coords(j,i) = floor(aux/d(i-1));
        aux = aux - coords(j,i)*d(i-1);
    end
    coords(j,1) = aux;
end

%Remove this line to get C++ mode coordinates (0 to n-1).
coords = coords+1; 