%% Dummy script to generate the barriers maps used in the benchmarking.

% Number of dimensions
% nd = 2;
nd = 3;

% Number of barriers
nbar = 9;

mkdir('maps');
if nd == 2
    height = 1000;
    width = 2000;

    for i = 0:nbar
        map = ones(width, height);
        % Odd barriers (top open)
        if i >= 1
            map(100:200, 1:height*0.9) = 0;
        end
        if i >= 3
            map(500:600, 1:height*0.9) = 0;
        end
        if i >= 5
            map(900:1000, 1:height*0.9) = 0;
        end
        if i >= 7
            map(1300:1400, 1:height*0.9) = 0;
        end
        if i >= 9
            map(1700:1800, 1:height*0.9) = 0;
        end

        % Even barriers (bottom open)
        if i >= 2
            map(300:400, height*0.1:end) = 0;
        end
        if i >= 4
            map(700:800, height*0.1:end) = 0;
        end
        if i >= 6
            map(1100:1200, height*0.1:end) = 0;
        end
        if i >= 8
            map(1500:1600, height*0.1:end) = 0;
        end

        map = map';
        imwrite(map, ['maps/2dbarriers_' num2str(i) '.png']);
        
    end
elseif nd == 3
    size = 100;
    height = 200;

    for i = 0:nbar
        map = ones(size, size, height);
        
        if i >= 1
            map(0.1*size:end, :, 10:20) = 0;
        end
        if i >= 2
            map(:, 0.1*size:end, 30:40) = 0;
        end
        if i >= 3
            map(1:0.9*size, :, 50:60) = 0;
        end
        if i >= 4
            map(:, 1:0.9*size, 70:80) = 0;
        end
        if i >= 5
            map(0.1*size:end, :, 90:100) = 0;
        end
        if i >= 6
            map(:, 0.1*size:end, 110:120) = 0;
        end
        if i >= 7
            map(1:0.9*size, :, 130:140) = 0;
        end
        if i >= 8
            map(:, 1:0.9*size, 150:160) = 0;
        end
        if i >= 9
            map(0.1*size:end, :, 170:180) = 0;
        end
    end

    saveGridToFile(map, ['maps/3dbarriers_' num2str(i) '.grid']);
end

%% Additional map plotting code.
% imagesc(map)
% axis xy;
% axis image;
% colormap gray(256);

% F = isosurface(map, 0.5);
% p = patch(F);
% isonormals(map, p); 
% view_ori = [-50 60];
% plot_map3d(map, 0.7, 1, view_ori, F);

% for i = 1:height
%     imagesc(map(:,:,i))
%     pause;
% end
