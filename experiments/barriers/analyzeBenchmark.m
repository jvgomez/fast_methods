%% This script assumes that fast/marching scripts folder is included in Matlab's
%  path.
clear all;
close all;

% This file assumes that all the experiments in path_to_benchmarks logged 
% the same algorithms in the same order. Variying the map, or the number of
% bars can result in strange plots, but it should be easy to adapt it.

%% Set the number of dimensions to analyze.
% nd = 2;
nd = 3;

%% Folders containing the results

path_to_benchmarks = [num2str(nd) 'd/results/'];
% path_to_errors = [num2str(nd) 'd_error/results/'];

%% Opening benchmark file.
files = [];
elements = dir(path_to_benchmarks);
for i = 3:size(elements,1)
    if ~elements(i).isdir
        files = [files; elements(i)];
    end
end

%% Parsing log files.
exps = cell(size(files,1),1);
nexps = size(files,1);
for i = 1:nexps
    exps{i} = parseBenchmarkLog([path_to_benchmarks files(i).name]);
end

%% Extracting the data we want.
nbars = zeros(1, nexps);
times = zeros(size(exps{1}.exp,1), nexps); % rows->algorithm, cols->experiment
algs = cell(1, size(exps{1}.exp,1));
for i = 1:size(algs,2)
    algs{i} = exps{1}.exp{i,1};
end

for i = 1:nexps
    nbars(i) = str2double(exps{i}.name(13));
    
    for j = 1:size(exps{i}.exp,1)
        times(j,i) = mean(exps{i}.exp{j,2});        
    end
end

%% Plotting
% Expected ordering: FMM, FMMFib, SFMM, GMM, FIM, UFMM, SFM, LSM, DDQM
markers = ['-+';'-o';'-*';'-<'; '-x';'-s';'-d';'-^';'->'];
colors = [[0 0 1]; [1 0 0]; [0 1 0]; [0 0 0]; [0 1 1]; [1 0 1]; [1 0.25 0.25]; ...
          [.5 .5 .5]; [.25 .25 1]];
figure;
hold on;
for i = 1:size(algs,2)
    plot(nbars, times(i,:), markers(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors(i,:));
end
legend(algs, 'Location', 'northwest');
xlabel('# Barriers');
ylabel('Time (ms)');

%% Analyzing errors for FMM, FIM and UFMM
% Assumes this ordering: 0001 FMM, 0002 FIM, 0003 UFMM
% files_err = [];
% dirs_err = [];
% elements_err = dir(path_to_errors);
% for i = 3:size(elements_err,1)
%     if ~elements_err(i).isdir
%         files_err = [files_err; elements_err(i)];
%     else
%         dirs_err = [dirs_err; elements_err(i)];
%     end
% end
% 
% L1 = zeros(2,size(dirs_err,1));
% Linf = zeros(2,size(dirs_err,1));
% 
% for i = 1:size(dirs_err,1)
%     % Load grids
%     grids_path = strcat(path_to_errors, dirs_err(i).name, '/');
%     fmm_grid = parseGrid(strcat(grids_path, '0001.grid'));
%     fim_grid = parseGrid(strcat(grids_path, '0002.grid'));
%     ufmm_grid = parseGrid(strcat(grids_path, '0003.grid'));
%     
%     % Compute errors
%     fim_err = fim_grid.cells - fmm_grid.cells;
%     ufmm_err = ufmm_grid.cells - fmm_grid.cells;
%     
%     % L1 norm is done with the integral (actually the mean of L1), see 
%     % Lp spaces for more info: http://en.wikipedia.org/wiki/Lp_space#Lp_spaces
%     % L1 = sum(|xi|*hx*hy) = sum(|xi|)*hx*hy (numerical integration.
%     L1(1,i) = norm(fim_err(:), 1) * fim_grid.leafsize^2;
%     L1(2,i) = norm(ufmm_err(:), 1) * ufmm_grid.leafsize^2;
%     
%     Linf(1,i) = norm(fim_err(:), Inf) * fim_grid.leafsize^2;
%     Linf(2,i) = norm(ufmm_err(:), Inf) * ufmm_grid.leafsize^2;
% end
% 
% %% Plotting
% markers_err = [markers(5,:); markers(6,:)]; % Positions of FIM and UFMM.
% colors_err = [colors(5,:); colors(6,:)];
% 
% figure;
% hold on;
% for i = 1:2
%     plot(ncells, L1(i,:), markers_err(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors_err(i,:));
% end
% for i = 1:2
%     plot(ncells, Linf(i,:), ['-',markers_err(i,:)], 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors_err(i,:));
% end
% algs_err = cell(1,4);
% algs_err{1} = 'FIM L_1';
% algs_err{2} = 'UFMM L_1';
% algs_err{3} = 'FIM L_\infty';
% algs_err{4} = 'UFMM L_\infty';
% h = legend(algs_err, 'Location', 'northwest');
% xlabel('# Cells');
% ylabel('L_1 Norm (s)');
