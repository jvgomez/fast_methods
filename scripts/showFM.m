% clear all; close all;

grid = parsegrid('../build/grid.txt');
    imagesc(grid.cells);
    hold on;
% path = parsepath('../build/test_path.txt');
%     plot(path(:,1),path(:,2),'k.');
%     axis xy;
%     hold off;
% pause;
% 
% grid = parsegrid('../build/test2_fm.txt');
% imagesc(grid.cells);
% axis xy;
% pause;
% 
% vels = parsegrid('../build/test_vels.txt');
% imagesc(vels.cells);
% colormap gray(256);
% axis xy;
% pause;
% 
% grid = parsegrid('../build/test_fm3d.txt');
%     plot_volumetric_data(grid.cells);
%     hold on;
% path = parsepath('../build/test_path3d.txt');
%     plot3(path(:,1),path(:,2), path(:,3),'k.');
%     pause;