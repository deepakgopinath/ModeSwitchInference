function [pg] = compute_p_of_g_dft_R2( uh, xr, pg )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    global ng delta_t;
    tau = 5;
    h = 1/ng;
    %%%%%%%%%%%%%%%%
    %only for ad mode
%     uh = aeval(uh);
    %%%%%%%%%%%%%%%%%%
    curr_pg = compute_curr_input(uh, xr);
    lambda = 0*ones(ng,ng);
    lambda(1: ng+1: ng*ng) = 0;
    lambda = 1*eye(ng) + lambda;
    dpgdt = (-1/tau)*eye(ng)*pg + (h/tau)*ones(ng,1) + lambda*sigmoid(curr_pg); %ODE - Dynamics neural field. 
    pg = pg + dpgdt*delta_t; %Euler integration;
    pg(pg <=0) = realmin;
    pg = pg/sum(pg); %normalize, so that the inidividual values are alwyas between 0 and 1. 
end

function out = sigmoid(u)
    out = 1./(1 + exp(-u));
    out = out - 0.5;
end

function curr_input = compute_curr_input(uh, xr)
    global ng xg;
    if size(xr, 2) > 1
        xr = xr';
    end
    curr_input = zeros(ng, 1);
    for i=1:ng %directedness based confidence function. 
        normxs = (xg(:, i) - xr)/(norm((xg(:, i) - xr)) + realmin); %ith goal position;
        normuh = uh/(norm(uh) +realmin); %add realmin to avoid divide by zero errors. 
        costh = dot(normxs, normuh);
        curr_input(i) = (1 + costh)/2;
    end
end