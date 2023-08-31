function  ACADO_main
clear all
%% your directory
addpath(genpath('/Users/user/Desktop/ACADO')) 

%% MPC sampling time
simTs = 0.1;           
%% MPC # of prediction horizon 
Num = 30;
DifferentialState x y z u v w phi theta ps phi_d theta_d psi_d force 
Control dphi_d dtheta_d dpsi_d dforce 

OnlineData max_U

OnlineData o1x
OnlineData o1y
OnlineData o1r

OnlineData o2x
OnlineData o2y
OnlineData o2r

OnlineData o3x
OnlineData o3y
OnlineData o3r

OnlineData floors
OnlineData obs_softa
OnlineData obs_softb

% physical parameters of airframe
P = load_parm();

R_z = [cos(ps), -sin(ps), 0;
       sin(ps), cos(ps), 0;
       0,0,1];
   
R_y = [cos(theta), 0, sin(theta);
       0, 1, 0;
      -sin(theta), 0, cos(theta)];
  
R_x = [1,0,0;
       0, cos(phi), -sin(phi);
       0, sin(phi), cos(phi)];
  
R = R_z*R_y*R_x;
e3 = -[0;0;1];
dv = P.gravity*e3 - (1/P.mass)*force*R*e3;

f = dot([x; y; z; u; v; w; phi; theta; ps; phi_d; theta_d; psi_d; force]) == ...
    [u;
     v;
     w;
     dv(1);
     dv(2);
     dv(3);
     (P.k_phi*phi_d-phi)/P.tau_phi;
     (P.k_theta*theta_d-theta)/P.tau_theta;
     (P.k_psi*psi_d-ps)/P.tau_psi;
     (dphi_d);
     dtheta_d;
     dpsi_d;
     dforce;
    ];
n_XD = length(diffStates);
n_U = length(controls);

eps = 1e-6;
obs1 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((x-o1x)^2/o1r^2 + (y-o1y)^2/o1r^2) )-1)));
obs2 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((x-o2x)^2/o2r^2 + (y-o2y)^2/o2r^2) )-1)));
obs3 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((x-o3x)^2/o3r^2 + (y-o3y)^2/o3r^2) )-1)));
obs = obs1+obs2+obs3;

h = [diffStates; controls;obs];
hN = [diffStates];

%% SIMexport (Integrator Setting)
acadoSet('problemname', 'sim');
sim = acado.SIMexport( simTs );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_RK4' );
sim.set( 'NUM_INTEGRATOR_STEPS',         2 );
ACADO_name = './../Quadrotor_NMPC';


%% MPCexport (optimal control setting)
acadoSet('problemname', 'NMPC');
ocp = acado.OCP( 0.0, simTs*Num, Num );
W_mat = eye(n_XD+n_U+1,n_XD+n_U+1);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);
ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo(  P.phi_min - phi_d <= 0 );
ocp.subjectTo(  phi_d - P.phi_max <= 0 );
ocp.subjectTo(  P.theta_min - theta_d <= 0 );
ocp.subjectTo(  theta_d - P.theta_max <= 0 );
ocp.subjectTo(  -pi - psi_d <= 0 );
ocp.subjectTo(  psi_d - pi <= 0 );
ocp.subjectTo(  z - floors >= 0 );
ocp.subjectTo(  u^2 + v^2 + w^2 - max_U^2 <= 0 );
ocp.setModel(f);


mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'PRINTLEVEL',                  'NONE'              );
mpc.set( 'INTEGRATOR_TYPE',             'INT_EX_EULER'      );
mpc.set( 'CG_USE_OPENMP ',              'YES');
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*Num                  );
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-8				);

% qpOASES dense solver
mpc.set( 'QP_SOLVER',           'QP_QPOASES'        );
mpc.set( 'SPARSE_QP_SOLUTION',  'FULL_CONDENSING_N2');
mpc.set( 'HOTSTART_QP',         'YES'             	);

mpc.exportCode( 'export_MPC' );
disp('qpOASES exported!');
% Your directory
copyfile('/Users/user/Desktop/ACADO/external_packages/qpoases','export_MPC/qpoases', 'f') 
cd export_MPC
    make_acado_solver(ACADO_name)
cd ..
end