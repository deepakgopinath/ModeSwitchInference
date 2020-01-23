function xr_new = sim_kinematics_R2(xr, uh) %all args are considered to be column vectors. 
    global delta_t;    
    xr_new = xr + uh.*delta_t + normrnd(0, 0.0000, length(xr), 1); %add stochasticity to the dynamics. 
end