%% This script assumes that fast/marching scripts folder is included in Matlab's
%  path.


%% Look for the comment "NEEDS TO BE CHANGED" if you change something in the
% experimental setup.
clear all;
close all;

% This file assumes that all the experiments in path_to_benchmarks logged 
% the same algorithms in the same order.

%% Set the number of dimensions to analyze.
% nd = 2;
% nd = 3;
nd = 4;

%% Folders containing the results
path_to_benchmarks = [num2str(nd) 'd/results/'];
path_to_errors = [num2str(nd) 'd_error/results/'];

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
vmin = zeros(1, nexps);
times = zeros(size(exps{1}.exp,1), nexps); % rows->algorithm, cols->experiment
algs = cell(1, size(exps{1}.exp,1));
for i = 1:size(algs,2)
    algs{i} = exps{1}.exp{i,1};
end

for i = 1:nexps
    vmin(i) = str2double(exps{i}.name(12:end));
    
    if vmin(i) == 0
        vmin(i) = 1;
    end
    
    for j = 1:size(exps{i}.exp,1)
        times(j,i) = mean(exps{i}.exp{j,2});        
    end
end

%% Manual restructuration of the vectors
%% NEEDS TO BE CHANGED IF THE EXPERIMENTS ARE CHANGED!
vmin = [vmin(1:2), vmin(4:end), vmin(3)];
times = [times(:,1:2), times(:,4:end), times(:,3)];

%% Plotting
% Expected ordering: FMM, FMMFib, SFMM, GMM, FIM, UFMM, SFM, LSM, DDQM
markers = ['-+';'-o';'-*';'-<'; '-x';'-s';'-d';'-^';'->'];
colors = [[0 0 1]; [1 0 0]; [0 1 0]; [0 0 0]; [0 1 1]; [1 0 1]; [1 0.25 0.25]; ...
          [.5 .5 .5]; [.25 .25 1]];
figure;
hold on;
for i = 1:size(algs,2)
    plot(vmin, times(i,:), markers(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors(i,:));
end
legend(algs, 'Location', 'northeast');
xlabel('Min. Velocity');
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
%     fmm_grid = parseGrid(strcat(grids_path, 'FMM.grid'));
%     fim_grid = parseGrid(strcat(grids_path, 'FIM.grid'));
%     ufmm_grid = parseGrid(strcat(grids_path, 'UFMM.grid'));
%     
%     % Compute errors
%     fim_err = fim_grid.cells - fmm_grid.cells;
%     ufmm_err = ufmm_grid.cells - fmm_grid.cells;
%     
%     % L1 norm is done with the integral (actually the mean of L1), see 
%     % Lp spaces for more info: http://en.wikipedia.org/wiki/Lp_space#Lp_spaces
%     % L1 = sum(|xi|*hx*hy) = sum(|xi|)*hx*hy (numerical integration), and
%     % hx ~ dom(X)/numX --> L1 = sum(|xi|)*dom(X)*dom(Y)/(numX*numY)
%     % where dom(X) is the real size of dimension X and numX the number of
%     % cells in that dimension.
%     domX = 1;
%     numX = size(fmm_grid.cells,1);
%     L1(1,i) = norm(fim_err(:), 1) * domX^nd  / (numX^nd); % Assuming cubic grid.
%     L1(2,i) = norm(ufmm_err(:), 1) * domX^nd  / (numX^nd);
%     
%     Linf(1,i) = norm(fim_err(:), Inf);
%     Linf(2,i) = norm(ufmm_err(:), Inf);
% end
% 
% %% Plotting
% markers_err = [markers(5,:); markers(6,:)]; % Positions of FIM and UFMM.
% colors_err = [colors(5,:); colors(6,:)];
% 
% figure;
% hold on;
% for i = 1:2
%     plot(vmin, L1(i,:), markers_err(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors_err(i,:));
% end
% for i = 1:2
%     plot(vmin, Linf(i,:), ['-',markers_err(i,:)], 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors_err(i,:));
% end
% algs_err = cell(1,4);
% algs_err{1} = 'FIM L_1';
% algs_err{2} = 'UFMM L_1';
% algs_err{3} = 'FIM L_\infty';
% algs_err{4} = 'UFMM L_\infty';
% h = legend(algs_err, 'Location', 'northeast');
% xlabel('Min. Velocity');
% ylabel('Norm (s)');
