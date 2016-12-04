%% This script assumes that fast/marching scripts folder is included in Matlab's
%  path.

%% Look for the comment "NEEDS TO BE CHANGED" if you change something in the
% experimental setup.
clear all;
close all;

% This file assumes that all the experiments in path_to_benchmarks logged 
% the same algorithms in the same order.
fs = 16;

%% Folders containing the results
path_to_benchmarks = ['times/results/'];
path_to_errors = ['error/results/'];

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
    vmin(i) = str2double(exps{i}.name(11:end));
    
    if vmin(i) == 0
        vmin(i) = 1;
    end
    
    for j = 1:size(exps{i}.exp,1)
        times(j,i) = mean(exps{i}.exp{j,2});        
    end
end

 % %% Analyzing errors for FMM, FIM and UFMM
 % % Assumes this ordering: 0001 FMM, 0002 UFMM, 0003 FIM
 % algs = 1;
  algs = 2; % If FIM analysis is also desired 
 
 files_err = [];
 dirs_err = [];
 elements_err = dir(path_to_errors);
 for i = 3:size(elements_err,1)
     if ~elements_err(i).isdir
         files_err = [files_err; elements_err(i)];
     else
         dirs_err = [dirs_err; elements_err(i)];
     end
 end
 
 L1 = zeros(algs,1);
 Linf = zeros(algs,1);
 
 for i = 1:1
     % Load grids
     fmm_grid = parseGrid(strcat(path_to_errors, 'FMM.grid'));
     ufmm_grid = parseGrid(strcat(path_to_errors, 'UFMM.grid'));
     fim_grid = parseGrid(strcat(path_to_errors, 'FIM.grid')); % Uncomment if
 %     FIM analysis is desired.
     
     % Compute errors
     ufmm_err = ufmm_grid.cells - fmm_grid.cells;
     fim_err = fim_grid.cells - fmm_grid.cells;
     
     % This gives some nan because of 0 velocities in the vessels.png image. Remove them.
     ufmm_err(isnan(ufmm_err)) = 0;
     fim_err(isnan(fim_err)) = 0;

     % L1 norm is done with the integral (actually the mean of L1), see 
     % Lp spaces for more info: http://en.wikipedia.org/wiki/Lp_space#Lp_spaces
     % L1 = sum(|xi|*hx*hy) = sum(|xi|)*hx*hy (numerical integration), and
     % hx ~ dom(X)/numX --> L1 = sum(|xi|)*dom(X)*dom(Y)/(numX*numY)
     % where dom(X) is the real size of dimension X and numX the number of
     % cells in that dimension.
     domX = 1;
     numX = size(fmm_grid.cells,1);
     L1(1,i) = norm(ufmm_err(:), 1) * domX^2  / (numX^2);
     L1(2,i) = norm(fim_err(:), 1) * domX^2  / (numX^2); % Assuming cubic grid.
     
     Linf(1,i) = norm(ufmm_err(:), Inf);
     Linf(2,i) = norm(fim_err(:), Inf);
 end
