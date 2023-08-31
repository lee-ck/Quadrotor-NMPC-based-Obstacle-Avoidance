function P = load_parm()    
% physical parameters of airframe
    P.gravity = 9.81;   % [m/s/s]
    P.mass    = 4.34;   % [kg]
    P.Jxx     = 0.0820; % [kg-m2]
    P.Jyy     = 0.0845; % [kg-m2]
    P.Jzz     = 0.1377; % [kg-m2]
    % The dist from CoM to the center of ea. rotor in the b1-b2 plane
    P.d  = 0.315; % [m]
    % actuator constant
    P.c_tauf = 8.004e-3; % [m]
    
    P.fmax = 40;
    P.fmin = 10;
    
    P.dfmax = 5;
    
    P.tau_phi = 0.1901;
    P.tau_theta = 0.1721;
    P.tau_psi = 0.1;

    P.k_phi = 0.91;
    P.k_theta = 0.96;
    P.k_psi = 0.90;
    
    P.phi_max = deg2rad(45);
    P.phi_min = deg2rad(-45);
    P.theta_max = deg2rad(45);
    P.theta_min = deg2rad(-45);
    
end