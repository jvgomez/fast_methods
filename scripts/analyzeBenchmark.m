clear all;
close all;

% This file assumes that all the experiments in path_to_benchmarks logged 
% the same algorithms in the same order.

%% Opening file.
%path_to_benchmarks = '../benchmark/build/results/';
path_to_benchmarks = 'results/';

files = dir(path_to_benchmarks);
files = files(3:end); 

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
figure;
plot(ncells, times, '-x', 'LineWidth', 2);
legend(algs, 'Location', 'northwest');