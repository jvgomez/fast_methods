%% This script assumes that fast/marching scripts folder is included in Matlab's
%  path, or uncomment next line
addpath('../../scripts/')

% This file assumes that all the experiments in path_to_benchmarks logged 
% the same algorithms in the same order.


%% Set the number of dimensions to analyze.
nd = 3;

fs = 16;

%% Folders containing the results
path_to_benchmarks = ['times/results/'];

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
ncells = zeros(1, nexps);
times = zeros(size(exps{1}.exp,1), nexps); % rows->algorithm, cols->experiment
algs = cell(1, size(exps{1}.exp,1));
for i = 1:size(algs,2)
    algs{i} = exps{1}.exp{i,1};
end

for i = 1:nexps
    ncells(i) = prod(exps{i}.dimsize);
    
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
    plot(ncells, times(i,:), markers(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors(i,:));
end
%lgnd = legend(algs, 'Location', 'northwest', 'FontSize', fs);
legend(algs, 'Location', 'northwest');

legend boxoff;
xlabel('# Cells', 'FontSize', fs);
ylabel('Time (ms)', 'FontSize', fs);
box on;
set(gca,'FontSize', fs);

saveas(gcf, [num2str(nd) 'emptymap.pdf'],'pdf');

%% Proportional plotting, with respect to FMM
ratios = bsxfun(@rdivide, times, times(1,:));
figure;
hold on;
for i = 1:size(algs,2)
    plot(ncells, ratios(i,:), markers(i,:), 'MarkerSize', 7, 'LineWidth', 1, 'Color', colors(i,:));
end
xlabel('# Cells', 'FontSize', fs);
ylabel('Ratio Time/FMM Time', 'FontSize', fs);
box on;
set(gca,'FontSize', fs);

% comentado porque en mi ordenador peta
% saveas(gcf, [num2str(nd) 'emptymap_ratio.pdf'],'pdf');
% para hacer por codigo
% print -painters -dpdf -r300 slices.pdf
