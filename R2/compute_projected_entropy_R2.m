function [ Upsilon ] = compute_projected_entropy_R2( uh, varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
projection_time = 1;
projected_pg = compute_pg_projection_R2(uh, projection_time, varargin{:});
Upsilon = compute_pg_entropy(projected_pg);
% Upsilon = compute_kl_div(projected_pg, varargin{3});
% Upsilon = compute_pg_entropy(projected_pg) + compute_kl_div(projected_pg,
% varargin{3}) % this boils down to only cross-entropy. 

end

