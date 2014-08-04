clear all; close all;

grid = parsegrid('../build/test_sfmm.txt');
    imagesc(grid.cells);
    axis xy;
    axis image;