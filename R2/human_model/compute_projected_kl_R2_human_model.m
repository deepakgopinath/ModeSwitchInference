function [ Upsilon ] = compute_projected_kl_R2_human_model( curr_mode, curr_goal, varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
global projection_time;
projected_pg = compute_pg_projection_R2_human_model(curr_mode, curr_goal, projection_time, varargin{:});
Upsilon = compute_kl_div(projected_pg, varargin{3});

end

