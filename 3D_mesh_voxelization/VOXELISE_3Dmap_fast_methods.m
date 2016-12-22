%% CONSTANTS

PLOT_MESH = 0;
CHECK_SIZE = 0;
view_ori = [38,10];

%% Plot the original STL mesh:
if PLOT_MESH
    [stlcoords] = READ_stl('townhall.stl','binary');
    xco = squeeze( stlcoords(:,1,:) )';
    yco = squeeze( stlcoords(:,2,:) )';
    zco = squeeze( stlcoords(:,3,:) )';
    [hpat] = patch(xco,yco,zco,'b');
    axis equal
    
    % Show the 2D projections of the mesh
    figure;
    subplot(1,3,1);
    imagesc(squeeze(sum(OUTPUTgrid,1)));
    colormap(gray(256));
    xlabel('Z-direction');
    ylabel('Y-direction');
    axis equal tight

    subplot(1,3,2);
    imagesc(squeeze(sum(OUTPUTgrid,2)));
    colormap(gray(256));
    xlabel('Z-direction');
    ylabel('X-direction');
    axis equal tight

    subplot(1,3,3);
    imagesc(squeeze(sum(OUTPUTgrid,3)));
    colormap(gray(256));
    xlabel('Y-direction');
    ylabel('X-direction');
    axis equal tight
end

%% TAMANHO DEL GRID de 2D y 3D ASUMIENDO LADOS IGUALES
dosD = [50, 100, 200, 400, 800, 1000, 1500, 2000, 2500, 3000, 4000].^2;
tresD = [14, 22, 34, 54, 86, 100, 131, 159, 184, 208, 252];

% Tamanho mesh
dim = [160,300,200];
% matriz de adaptacion del tamaho para lados distintos
XYZ = zeros(size(tresD,2),3);

% hard coded values to re-arrage the size of dimensions
multiplicador = [dim/225; dim/220; dim/215; dim/215; dim/213; dim/214; dim/213; dim/213; dim/213; dim/213; dim/213 ];
if CHECK_SIZE
    for i=1:1:size(tresD,2)
        XYZ(i,:) = ceil(tresD(i).*multiplicador(i,:));
        comparacion(i) = dosD(i)/(XYZ(i,1)*XYZ(i,2)*XYZ(i,3));
    end
% El resultado debe ser lo más parecido posible a 1.0
comparacion
end


%% compute each grid
meshXYZ = READ_stl('townhall.stl');
filename = 'grid_';
for i=1:1:size(tresD,2)
    XYZ = ceil(tresD(i).*multiplicador(i,:));
    [OUTPUTgrid] = VOXELISE(XYZ(1),XYZ(2),XYZ(3),meshXYZ,'xyz');
%     saveGridToFile(OUTPUTgrid, strcat(filename,num2str(tresD(i))))
    save(strcat(strcat(filename,num2str(tresD(i))),'.mat'), 'OUTPUTgrid');
%     if PLOT_MESH
%         figure(3+i)
%         plot_map3d(OUTPUTgrid, 0.2, 3+i, view_ori);   
%     end
plot_map3d(OUTPUTgrid, 0.2, 1, view_ori);   
pause
clf(1)
end

