clear all; close all; clc;
%%

%% SPAWN GOAL CONFIGURATION AND ROBOT POSITION. RANDOMLY IN THE WORKSPACE. 

%Define workspace limits. All in metres. 
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];

%global variables.
global cm num_modes ng nd xg xr sig delta_t conf_thresh conf_max num_samples alpha_max sparsity_factor amp_sparsity_factor kappa projection_time;

%workspace parameters
max_ng = 1;
ng = datasample(1:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 2
nd = 2; %num of dimensions. by definition R^2
cm = {1,2}; %only control mode settings. 1D modes. 
num_modes = length(cm); %or 1 depending on whether the modes are [x, y] or [{x,y}]
init_mode_index = datasample(1:num_modes, 1);

%spawn random goals and random robot positions
xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1)];
xr_true = xr; %store away random  robot position in a variable for reuse.

%human parameters
sparsity_factor = rand/8; %how often would the human control command be zero.
amp_sparsity_factor = rand/8; % how often the amplitude wiull be less that maximum. 
kappa = 20.0; % concentration paarameter for vonMisesFisher distribution
fprintf('The sparsity and amp factor are %f, %f\n', sparsity_factor, amp_sparsity_factor);

intended_mode_switch_action_to_interface_action_error_rate = 0.0; %capture the error that can happen due to lack of skill
mode_error_rate = 0.0; %rate at which a correct mode switch gets disrupted. 

%% Projection paramaters
projection_time = 4;
delta_t = 0.1; %For compute projections.
num_samples = 5;

%% simulation params
mode_comp_timesteps = 5; %time step gap between optimal mode computation. delta_t*mode_comp_timesteps is the time in seconds
exit_threshold = 0.02;
total_time_steps = 120; %with delta_t of 0.1, this amounts to 10 seconds. We will assume that "mode switches" don't take time. 

%%

%arbitrartion function parameters - Fixed parameters for shared control. 
conf_thresh = (1.05/ng);
conf_max = (1.1/ng);
alpha_max = 0.7;

%% PLOT GOALS AND CURRENT ROBOT POSITION. 
% 
figure;
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled'); grid on; hold on;
scatter(xr(1), xr(2), 140, 'r', 'filled');
offset = [-0.1, 0.1];
line(xrange+offset, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange+offset, 'Color', 'g');
axis([xrange + offset, yrange + offset]);
axis square;


%% generate random user goal index. change color to magenta. 
random_goal_index = randsample(ng, 1); 
random_goal = xg(:, random_goal_index);

%% %% USING MAX POTENTIAL AS BASE LINE
%variables to hold simulation data for MAX POTENTIAL
pgs_POT = zeros(ng, total_time_steps); %goal probabilities
optimal_modes_POT = zeros(total_time_steps-1, 1); %optimal modes. 
alpha_POT = zeros(total_time_steps-1, 1); %alpha values
uh_POT = zeros(nd, total_time_steps-1);
ur_POT = zeros(nd, total_time_steps-1);
blend_vel_POT = zeros(nd, total_time_steps-1);
curr_goal_POT = zeros(total_time_steps-1, 1); %current goal inference
traj_POT = zeros(nd, total_time_steps); %trajectory
traj_POT(:, 1) = xr; %initialize first data point of trajetcory
pgs_POT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory

current_optimal_mode_POT_index = init_mode_index;
current_optimal_mode_POT = cm{current_optimal_mode_POT_index};

for i=1:total_time_steps-1
    %compute the optimal mode. store it away. 
    if norm(traj_POT(:, i) - random_goal)/norm(traj_POT(:, 1)- random_goal) < exit_threshold
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_POT = 1;
    curr_goal_POT(i) = curr_goal_index_POT;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_POT_index = compute_optimal_mode_POT_R2(xg(:, curr_goal_index_POT), xr_true); 
        if length(current_optimal_mode_POT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_POT = cm{current_optimal_mode_POT_index(1)}; %pick the first one. 
            current_optimal_mode_POT_index = current_optimal_mode_POT_index(1); %update the index. 
        else
            if current_optimal_mode_POT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_POT = cm{current_optimal_mode_POT_index}; 
            end
        end
    end
    %gotta determine uh. Assumes human is executes the "straightest
    %possible motion in the current mode towards the specified goal". 
    uh = generate_full_uh(random_goal, xr); %noisy uh. Change gaussian sigma in the generate_full_uh accordingly. 
    zero_dim = setdiff(1:nd,current_optimal_mode_POT); %zero out the nonavailable control modes. 
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;   
    end
    %normalize
    if norm(uh) > 0.2
        uh = 0.2*(uh./(norm(uh) + realmin));
    end
%     if rand < amp_sparsity_factor
%         uh = rand*uh;
%     end
    %human and robot control commands and belnding
%     ur = generate_autonomy(curr_goal_index_POT); %pfield autonomy command in full 2D space towards what it thinks is the current goal
%     alpha_POT(i) = alpha_from_confidence(pgs_POT(curr_goal_index_POT, i)); %linear belnding param
%     blend_vel = (1-alpha_POT(i))*uh + alpha_POT(i)*ur; %blended vel
%     blend_vel = uh;
    uh_POT(:, i) = uh; 
%     ur_POT(:, i) = ur; 
%     blend_vel_POT(:, i) = blend_vel;
%     if strcmp(intent_type, 'dft')
%         pgs_POT(:, i+1) = compute_p_of_g_dft_R2(uh, xr, pgs_POT(:, i));
%     elseif strcmp(intent_type, 'bayes')
%         pgs_POT(:, i+1) = compute_bayes_R2(uh, xr, pgs_POT(:, i));
% %         pgs_POT(:, i+1) = compute_bayes_n_R2(uh_POT(:, max(1, i-hist_length+1):i), xr, pgs_POT(:,i));
%     elseif strcmp(intent_type, 'conf')
%         pgs_POT(:, i+1) = compute_conf_R2(uh, xr);
%     end
    xr = sim_kinematics_R2(xr, uh); %forward simulate kinematics with blending. 
    traj_POT(:, i+1) = xr;
    optimal_modes_POT(i) = current_optimal_mode_POT_index;
end

%% PLOT TRAJECTORIES
hold on; 
scatter(traj_POT(1, :)', traj_POT(2, :)', 'k', 'filled');

%% generate u_h
function uh = generate_full_uh(xg, xr) %full unnomralized uh
    global nd sparsity_factor kappa;
    mu = xg - xr;
    if ~any(mu)
        uh = zeros(nd, 1);
    else
        uh = randvonMisesFisherm(nd, 1, kappa, mu);
    end
   
    %add sparsity
    if rand < sparsity_factor
        uh = zeros(nd, 1);
    end
    
end



%%
function [ best_mode ] = compute_optimal_mode_POT_R2(xg, xr_true)
    global cm nd xr;
    Pk = zeros(nd, 1);
    Pcm = zeros(length(cm), 1); % Fractional Potnetial for each mode. 
    for i=1:nd
        Pk(i) = abs((xg(i) - xr(i))/(xg(i) - xr_true(i))); %Fractional potential. 
    end
    for i=1:length(cm)
        Pcm(i) = sum(Pk(cm{i}));
    end
    best_mode = compute_best_mode(Pcm);
end
