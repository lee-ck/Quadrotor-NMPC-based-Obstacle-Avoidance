clear all
P = load_parm();

figure(1)
clf;
hold on
grid on
box on
set(gcf,'color','white');
axis equal
view([52 22]) % normal view
view([0 90]) % top view
view([90 0]) % side view

aa = plot_quad_obj([0,0,0]',[0,0,0],1);
bb = plot3([0,1],[0,1],[0,1]);
cc = plot3([0,1],[0,1],[0,1]);

tps =  [3,3,5;...
        3,3,5;...
        5,7,5;...
        7,3,5;...
        7,7,5;...
        5,3,5;...
        3,7,5];      
    
tps = repmat(tps,3,1);

tp_index = 1;
tp = tps(tp_index,:);
position = tp;
velocity = [0,0,0];
R = eye(3);
R = reshape(R,[1,9]);
rot = [0,0,0];
Omega = [0,0,0];
states = [position, velocity, rot, 0,0,0,43];
input = [0,0,0,0];
   
tfinal = 65;
dt = 0.1;
save_data = [];
radius = 0.6;
[xq yq zq] = sphere(22);
hq = surfl(xq*radius, yq*radius, zq*radius); 
set(hq, 'FaceColor','r', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1)

% Create a grid for the base of the cylinder in the xy-plane
theta = linspace(0, 2*pi, 100);
z = linspace(0, 6.8, 100);
[Theta, ZE] = meshgrid(theta, z);


%% Acado
Num = 30; Ts = 0.1;
nx = 13; nu = 4;
% x y z u v w R11 R12 R13 R21 R22 R23 R31 R32 R33 roll pitch yaw
% f1 f2 f3 f4
MPC.x0 = zeros(1,nx);
MPC.x = zeros(Num+1,nx);
MPC.u = zeros(Num,nu);
MPC.od = zeros(Num+1,13); 
MPC.y = zeros(Num,nx+nu+1);
MPC.yN = zeros(1,nx); 

MPC.W = diag([.2,... %  x
              .2,... %  y
              .2,... %  z
              1.0,... %  u
              1.0,... %  v
              1.0,... %  w
              10.0,... %  roll
              10.0,... %  pitch
              10.0,... %  yaw
              0.0,... %  phi_d
              0.0,... %  theta_d
              0.0,... %  psi_d
              0,... %  force
              1,... %  dphi_d
              1,... %  dtheta_d
              1,... %  dpsi_d
              0.1,... %  dforce
              1]); %  ostacle potential
          
MPC.WN = MPC.W(1:nx,1:nx)*Num;
          

Record = 0;
if Record
    vv = VideoWriter('test');
    open(vv);
end
plotcube([1,1,0],2,2,4,0,0,0,[1,0,0],0.2)

for t = 0:dt:tfinal-dt
    MPC.od(:,1) = 2; % Max speed
    s1 = 2;
    s2 = 2;
    s3 = 1;
    
    obs = [5+s1*cos(t/3), 5+s1*sin(t/3),  1.1;...
           7, 5, 0.8;...
           4, 5, 0.8];
   
    for jj = 1:Num+1
        MPC.od(jj,2) = 5+s1*cos((t+(jj-1)*Ts)/3);
        MPC.od(jj,3) = 5+s1*sin((t+(jj-1)*Ts)/3);
        MPC.od(jj,4) = obs(1,3);

        MPC.od(jj,5) = obs(2,1);
        MPC.od(jj,6) = obs(2,2);
        MPC.od(jj,7) = obs(2,3);
        
        
        MPC.od(jj,8) = obs(3,1);
        MPC.od(jj,9) = obs(3,2);               
        MPC.od(jj,10) = obs(3,3);    
    end
    MPC.od(:,11) = 4; % floor
    MPC.od(:,12) = 5; % soft constraint
    MPC.od(:,13) = 10; % soft constraint
   
    if t==0   
        for jj = 1:3        
            x = (obs(jj,3))*cos(Theta) + obs(jj,1);
            y = (obs(jj,3))*sin(Theta) + obs(jj,2);
            pp(jj) = surf(x,y,ZE);                    
            set(pp(jj), 'FaceColor','b', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1)        
        end
    end
    dummys = 0;
    MPC.x0 = [states];
    if mod(t,7) < 0.001
        tp_index = tp_index + 1;
        tp = tps(tp_index,:);
        plot3(tp(1),tp(2),tp(3),'rx','LineWidth',5) 
    end
    
    if t >= 50
        tp = [2,2,4];
        if t == 50
            plot3(tp(1),tp(2),tp(3),'rx','LineWidth',5) 
        end
    end
    
    MPC.y(:,1) = tp(1);
    MPC.y(:,2) = tp(2);
    MPC.y(:,3) = tp(3);
    MPC.yN(:,1) = tp(1);
    MPC.yN(:,2) = tp(2);
    MPC.yN(:,3) = tp(3);
    if t == 0
        plot3(tp(1),tp(2),tp(3),'rx','LineWidth',5) 
    end
    iterNumth = 0;
    tic
    if t>0
        if output.info.kktValue>1e3
            MPC.x = repmat(MPC.x0,Num+1,1);
            MPC.u = zeros(Num,nu);
        end
    end
    while iterNumth < 50
        MPC.x(:,7) = ssa(MPC.x(:,7));
        MPC.x(:,8) = ssa(MPC.x(:,8));
        MPC.x(:,9) = ssa(MPC.x(:,9));
        MPC.x(:,10) = ssa(MPC.x(:,10));
        MPC.x(:,11) = ssa(MPC.x(:,11));
        MPC.x(:,12) = ssa(MPC.x(:,12));
        output = Quadrotor_NMPC(MPC);
        MPC.x = [output.x(2:end,:); output.x(end,:)];
        MPC.u = [output.u(2:end,:); output.u(end,:)];
        iterNumth = iterNumth + 1;
        if output.info.kktValue < 1e-08
            break;
        end
    end
    tt = toc;
    input = output.u(1,:);
    
    if abs(input(4)) > P.dfmax*4
        input(4) = P.dfmax*4*sign(input(4));
    end
    if abs(input(1)) > P.phi_max
        input(1) = P.phi_max*sign(input(1));
    end
    if abs(input(2)) > P.theta_max
        input(2) = P.theta_max*sign(input(2));
    end
    if abs(input(3)) > pi
        input(3) = pi*sign(input(3));
    end    
    
    [ds] = quad_dynamics(states,input);

   
    states = states + ds*dt;
    save_data = [save_data; t, states, input, reshape(obs',1,9), output.info.kktValue, tt];
    
    if mod(t,1) < 1
        delete(aa)
        delete(bb)
        delete(cc)
        delete(hq)
        
        for jj = 1:3
            delete(pp(jj))
        end
        for jj = 1:3
            x = (obs(jj,3)-radius)*cos(Theta) + obs(jj,1);
            y = (obs(jj,3)-radius)*sin(Theta) + obs(jj,2);
            pp(jj) = surf(x,y,ZE);                    
            set(pp(jj), 'FaceColor','b', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1)
        end
        
        hq = surfl(xq*radius+states(1), yq*radius+states(2), zq*radius+states(3)); 
        set(hq, 'FaceColor','r', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1)
        aa = plot_quad_obj(states(1:3)',states(7:9));
        bb = plot3(save_data(:,2),save_data(:,3),save_data(:,4),'k--','LineWidth',1.5);
        cc = plot3(output.x(:,1),output.x(:,2),output.x(:,3),'m--','LineWidth',1.5);
        xlim([0 10])
        ylim([0 10])
        zlim([3 7])
        drawnow
        if Record
            frame = getframe(gcf);
            writeVideo(vv,frame);
        end
    end
end


if Record
    close(vv);
end


plot_save_data

