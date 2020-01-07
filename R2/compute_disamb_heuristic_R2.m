function [ Dk ] = compute_disamb_heuristic_R2(uh, varargin )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
projection_time = 1; %project for one second, get projected probability
projected_pg_1 = compute_pg_projection_R2(uh, projection_time, varargin{:});
projection_time = 2; %project for two second, get projected probability
projected_pg_2 = compute_pg_projection_R2(uh, projection_time, varargin{:});
Dk = compute_disamb_metric(projected_pg_1, projected_pg_2);
end

