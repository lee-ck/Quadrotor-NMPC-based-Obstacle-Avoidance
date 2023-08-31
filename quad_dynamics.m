function [sys] = quad_dynamics(x,uu)
    P = load_parm();
    p       = x(1:3);
    v       = x(4:6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    phi_d = x(10);
    theta_d = x(11);
    psi_d = x(12);
    f = x(13);
 
    dphi_d = uu(1);
    dtheta_d = uu(2);
    dpsi_d = uu(3);
    df = uu(4);
    
    
    % state derivative vector
    xdot = zeros(size(x));
    e3 = -[0;0;1];
    xdot(1:3) = v;
   
    R_z = [cos(psi), -sin(psi), 0;
           sin(psi), cos(psi), 0;
           0,0,1];

    R_y = [cos(theta), 0, sin(theta);
           0, 1, 0;
          -sin(theta), 0, cos(theta)];

    R_x = [1,0,0;
           0, cos(phi), -sin(phi);
           0, sin(phi), cos(phi)];
    R = R_z*R_y*R_x;

    xdot(4:6) = P.gravity*e3 - (1/P.mass)*f*R*e3 + [0.3;0.3;0]*0;
    xdot(7) = (P.k_phi*phi_d-phi)/P.tau_phi;
    xdot(8) = (P.k_theta*theta_d-theta)/P.tau_theta;
    xdot(9) = (P.k_psi*psi_d-psi)/P.tau_psi;
    xdot(10) = dphi_d;
    xdot(11) = dtheta_d;
    xdot(12) = dpsi_d;
    xdot(13) = df;
    sys = xdot;
end
