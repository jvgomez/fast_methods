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

fs = 16;

%% Folders containing the results
path_to_benchmarks = [num2str(nd) 'd/results/'];

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
legend(algs, 'Location', 'northwest','FontSize', fs);
legend boxoff
xlabel('# Barriers','FontSize', fs);
ylabel('Time (ms)','FontSize', fs);
set(gca,'FontSize', fs);
box on;
saveas(gcf, [num2str(nd) 'barriers.pdf'],'pdf');

%% Proportional plotting, with respect to FMM
ratios = bsxfun(@rdivide, times, times(1,:));
figure;
hold on;
for i = 1:size(algs,2)
    plot(nbars, ratios(i,:), markers(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors(i,:));
end
xlabel('# Cells', 'FontSize', fs);
ylabel('Ratio Time/FMM Time', 'FontSize', fs);
box on;
set(gca,'FontSize', fs);

saveas(gcf, [num2str(nd) 'barriers_ratio.pdf'],'pdf');

