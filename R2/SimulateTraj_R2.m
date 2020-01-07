clear all; close all; clc;

%% SPAWN GOAL CONFIGURATION AND ROBOT POSITION. RANDOMLY IN THE WORKSPACE. 

%Define workspace limits. All in metres. 
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];

global num_modes ng nd xg xr sig delta_t;

ng = 5; %num of goals
nd = 2; %num of dimensions. by definition R^2
num_modes = 2; %or 1 depending on whether the modes are [x, y] or [{x,y}]

xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1)];
xr_true = xr;
% xr = [0;0];

sig = 0.01; %For Fisher information
delta_t = 0.1; %For compute projections. 

%% PLOT GOALS AND CURRENT ROBOT POSITION. 

figure;
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled'); grid on; hold on;
scatter(xr(1), xr(2), 140, 'r', 'filled');
for i=1:ng %vectors connecting robot and goals.
    quiver(xr(1), xr(2), xg(1,i) - xr(1), xg(2,i) - xr(2), 'LineWidth', 1.5, 'LineStyle', '-.');
end
offset = [-0.1, 0.1];
line(xrange+offset, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange+offset, 'Color', 'g');
axis([xrange + offset, yrange + offset]);
axis square;

%% START SIMULATING A HUMAN CONTROLLING THE ROBOT FROM THE CURRENT POSITION ALWAYS IN THE DISAMBIGUATING MODE, TOWARDS A RANDOMLY PICKED GOAL FROM 
%% THE LIST OF GOALS.

random_goal_index = randsample(ng, 1);
random_goal = xg(:, random_goal_index);
scatter(random_goal(1), random_goal(2), 180, 'm', 'filled'); grid on; hold on;

%%
disamb_type = 'ent'; %or dk or fi
intent_type = 'dft'; % or conf or bayes
interface_type = '1d'; %or 2d or 3d
features_list = 'd'; %d for directedness, p for proximity, a for agreement (rotation only). 

%% simulate motion towards random goal from starting xr

total_time_steps = 100; %with delta_t of 0.1, this amounts to 10 seconds. We will assume that "mode switches" don't take time. 
pgs_ENT = zeros(ng, length(total_time_steps));
optimal_modes_ENT = zeros(length(total_time_steps), 1);
traj_ENT = zeros(nd, length(total_time_steps));
traj_ENT(:, 1) = xr;
pgs_ENT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection
current_optimal_mode_ENT = 1;
current_optimal_mode_FI = 1;
mode_comp_timesteps = 10;
for i=1:total_time_steps-1
    %compute the optimal mode. store it away. 
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_ENT = compute_optimal_mode_ENT_2D(intent_type, xr, pgs_ENT(:, i), interface_type);
        if current_optimal_mode_ENT == 3
            current_optimal_mode_ENT = 1;
        end
    end
    
    %gotta determine uh
    uh = random_goal - xr;
    zero_dim = setdiff(1:nd,current_optimal_mode_ENT);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     if norm(uh) < 0.2*delta_t
%         break;
%     end
    uh = 0.2*(uh/(norm(uh) + realmin));
    pgs_ENT(:, i+1) = compute_p_of_g_dft(uh, xr, pgs_ENT(:, i));
    xr = sim_dyn(xr, uh);
    traj_ENT(:, i+1) = xr;
    optimal_modes_ENT(i) = current_optimal_mode_ENT;
end
%% USING FI
xr = xr_true;
pgs_FI = zeros(ng, length(total_time_steps));
optimal_modes_FI = zeros(length(total_time_steps), 1);
traj_FI = zeros(nd, length(total_time_steps));
traj_FI(:, 1) = xr;
pgs_FI(:, 1) = (1/ng)*ones(ng, 1);
for i=1:total_time_steps-1
    %compute the optimal mode. store it away. 
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_FI = compute_optimal_mode_FI_2D(intent_type, xr, pgs_FI(:, i), interface_type);
        if current_optimal_mode_FI == 3
            current_optimal_mode_FI = 1;
        end
    end
    %gotta determine uh
    uh = random_goal - xr;
    zero_dim = setdiff(1:nd,current_optimal_mode_FI);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     if norm(uh) < 0.2*delta_t
%         break;
%     end
    uh = 0.2*(uh/(norm(uh) + realmin));
    pgs_FI(:, i+1) = compute_p_of_g_dft(uh, xr, pgs_FI(:, i));
    xr = sim_dyn(xr, uh);
    traj_FI(:, i+1) = xr;
    optimal_modes_FI(i) = current_optimal_mode_FI;
end
%%
hold on; 
scatter(traj_ENT(1, :)', traj_ENT(2, :)', 'r', 'filled');
scatter(traj_FI(1, :)', traj_FI(2, :)', 'b', 'filled');

%% PLOT PROBABILITIES AND THE DISAMB MODES. 
figure; 
subplot(2,2,1);
plot(pgs_ENT','LineWidth', 1.5); grid on;
ylim([0.1, 0.8]);
title('ENTROPY PG'); xlabel('Time Steps'); ylabel('PG')
subplot(2,2,2);
plot(pgs_FI', 'LineWidth', 1.5); grid on;
title('FI PG'); xlabel('Time Steps'); ylabel('PG')
ylim([0.1, 0.8]);


subplot(2,2,3);
scatter(1:length(optimal_modes_ENT), optimal_modes_ENT, 'r', 'filled'); 
title('ENTROPY MODES');
subplot(2,2,4);
scatter(1:length(optimal_modes_FI), optimal_modes_FI, 'b', 'filled');
title('FI MODES');
color_scheme = {'b', 'r', 'y', 'pur', 'green'};
fprintf('THE RANDOM GOAL IS %d and COLOR is %s\n', random_goal_index, color_scheme{random_goal_index});
fprintf('THE NUMBER OF MODES SWITCHES ENT %d AND FI %d\n',sum(abs(diff(optimal_modes_ENT))), sum(abs(diff(optimal_modes_FI))));
%b,r,y, purple, green