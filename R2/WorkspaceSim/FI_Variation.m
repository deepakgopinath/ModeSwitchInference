clear all; clc; close all;
%%
%Define workspace limits. All in metres. 
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];

global cm num_modes ng nd xg xr sig delta_t conf_thresh conf_max alpha_max;

%workspace parameters
max_ng = 3;
ng = datasample(3:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 2; %num of dimensions. by definition R^2
cm = {1,2}; %only control mode settings. 
num_modes = length(cm); %or 1 depending on whether the modes are [x, y] or [{x,y}]

%spawn random goals and random robot positions
xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1)];
xr_true = xr; %store away random position in a variable for reuse.
xg(2, :) = 0.3*ones(1,ng); %make y axis all 0.1; 


%disambiguation related params
sig = 0.01; %For Fisher information
delta_t = 0.1; %For compute projections. 

%arbitrartion function parameters - Fixed parameters for shared control. 
conf_thresh = (1.2/ng);
conf_max = (1.4/ng);
alpha_max = 0.7;

%%

%% PLOT GOALS AND CURRENT ROBOT POSITION. 

figure;
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled'); grid on; hold on;
scatter(xr(1), xr(2), 140, 'r', 'filled');
offset = [-0.1, 0.1];
line(xrange+offset, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange+offset, 'Color', 'g');
axis([xrange + offset, yrange + offset]);
axis square;

%%
intent_type = 'dft';

%%

min_ws = -0.5;
max_ws = 0.5;
num_steps = 100;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)'; %so that it doesnt align exactly with the goals?
[X,Y] = meshgrid(ax_p);
ws_points = [X(:) Y(:)]; ws_points(:, 2) = zeros(size(ws_points, 1), 1); ws_points = unique(ws_points); ws_points = [ws_points, zeros(size(ws_points, 1), 1)];%make y coordinate to be zero. 
disamb_modes_ENT = zeros(size(ws_points, 1), 1);
disamb_modes_FI = zeros(size(ws_points, 1), 1);
pg0 = (1/ng)*ones(ng,1); %at every workspace point the starting probability distribution is uniform. 
% pg0 = rand(ng, 1); pg0 = pg0/sum(pg0);
current_optimal_mode_FI = cm{datasample(1:num_modes, 1)};
EID_AR_list = zeros(nd, size(ws_points, 1));
%%
for i=1:size(ws_points,1)
    xr = ws_points(i, :)';
    %%FI:
    [current_optimal_mode_FI_index, EID_AR] = compute_optimal_mode_FI_R2(intent_type, xr, pg0); 
    EID_AR_list(:, i) = EID_AR;
    if length(current_optimal_mode_FI_index) > 1 %when there are equivalent modes. 
        current_optimal_mode_FI = cm{current_optimal_mode_FI_index(1)}; %pick the first one. 
        current_optimal_mode_FI_index = current_optimal_mode_FI_index(1); %update the index. 
    else
        if current_optimal_mode_FI_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
            current_optimal_mode_FI = cm{current_optimal_mode_FI_index}; 
        end
    end
    

    disamb_modes_FI(i) = current_optimal_mode_FI; 
    if current_optimal_mode_FI_index == -1
        disamb_modes_FI(i) = 3;
    end
end
%%
%%
figure;grid on; hold on;
colors = {[1,0,0], [0,1,0], [0,0,1]};

for i=1:length(colors)
    scatter(ws_points(disamb_modes_FI == i, 1), ws_points(disamb_modes_FI == i, 2), 70, colors{i}, 'filled'); hold on;
end
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled');
% xrange = [-5.5,5.5]; %set axis limits
% yrange = [-5.5,5.5];
line(xrange, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange, 'Color', 'g');
axis([xrange, yrange]);
legend('Mode 1', 'Mode 2', 'Equivalent');
axis square;
xlabel('Spatial X'); ylabel('Spatial Y'); title('Best Disamb Control Modes');
title('FI BASED METRIC')

%%
figure;
plot(ws_points(:, 1), EID_AR_list(1, :)/max(EID_AR_list(1, :)), 'r')
grid on; hold on;
% plot(ws_points(:, 1), EID_AR_list(2, :), 'b')
plot(ws_points(:, 1), EID_AR_list(2, :)/max(EID_AR_list(2, :)), 'b');
% for i=1:length(colors)
%     scatter(ws_points(disamb_modes_FI == i, 1), ws_points(disamb_modes_FI == i, 2), 70, colors{i}, 'filled'); hold on;
% end