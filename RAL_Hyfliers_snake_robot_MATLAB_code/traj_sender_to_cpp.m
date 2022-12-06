clear all
close all
clc

addpath('Data')
addpath('prisma_snake_functions')


%% -----------------------ROS INTERCONNECTION -----------------------------
rosshutdown
setenv('ROS_MASTER_URI','http://localhost:11311')
setenv('ROS_IP','localhost')
rosinit

traj_setpoints_pub = rospublisher('/prisma_snake/traj','std_msgs/Float64MultiArray');


%% -------------------------- DYNAMIC PARAMETERS --------------------------
% Loading dynamic parameters of the manipulator from param.m scripts
param

%% ------------------------ TRAJETORY PLANNING .---------------------------
N_points = 300;         % Number of points in the trajectory
N_additional = 30;      % Additional points to check dynamic evolution without inputs
Ti = 0;                 % Starting time 
Tf = 3;                 % Ending time

load('sens_theta3');
theta_init = sens_theta;
theta_init(2) = 0.0; 
T_WUAV_init = [eul2rotm([0.0 0.0 -0.1745]) [2 -1.185, 2.285]'; 0 0 0 1];	
T_start = snake_dirkin(theta_init, parameters);     

% Final position
load('sens_theta3');
theta_goal = sens_theta;
T_goal = snake_dirkin(theta_goal, parameters);     

% Trajectory generation in Cartesian Space
R_des = [];
p_des = [];
pd_des = [];
R_eff = [];
p_eff = [];

[T_traj, Td_traj, omega_des, pdd_des, omegad_des] = traj_generation(T_start, T_goal, Ti, Tf, N_points);

T_traj_t = T_traj;
Td_traj_t = Td_traj;

for i=1:N_points
    R_des(:,:,i) = T_traj(1:3,1:3,i);
    p_des(:,i) = T_traj(1:3,4,i);
    
    pd_des(:,i) = Td_traj(1:3,4,i);
end

%% Artificial potential
robot_radius = 0.055/2;   
                                 
q0 = [-0.346704042782106  0.031953011496208 0.0];    
qg = [-0.43 -0.55 0.00];
qg_1 = [-0.27,-0.24, 0.00];
qg_2 = [-0.48,-0.07, 0.00];
qg_3 = [-0.65,-0.12, 0.00];
qg_4 = [-0.12,-0.4, 0.00];

box_x = fix(0*1000)/1000;                               % Obtaining position of the workspace
box_y = fix(0*1000)/1000;
box_th = 0.0;
box_len = 2.0;                                          % Length of the square workspace
box_len_aug = box_len - 2*robot_radius -0.04;           % Augmented length of the square obstacle

cyl1_x = fix(0.0*1000)/1000;                            % Obtaining position of the cylindrical obstacle n°1
cyl1_y = fix(-0.233674*1000)/1000;
cyl1_th = 0.0;
r1 = 0.136728;                                          % Radius of obstacle n°1
r1_aug = r1 + robot_radius + 0.01;                      % Augmented radius of the obstacle n°1

cyl2_x = fix(-0.48*1000)/1000;                          % Obtaining position of the cylindrical obstacle n°2
cyl2_y = fix(-0.233674*1000)/1000;
cyl2_th = 0.0;
r2 = 0.136728;                                          % Radius of obstacle n°2
r2_aug = r2 + robot_radius + 0.01;                      % Augmented radius of the obstacle n°2

% Matrix with center of each element in the scene
cc = [box_x, box_y, box_th; cyl1_x, cyl1_y, cyl1_th; cyl2_x, cyl2_y, cyl2_th];   

[~, ~, ~, points_path] = navigation_function(q0,[-0.485, -0.45,0.00],cc, box_len , box_len_aug, r1_aug, r2_aug); 

y_y = points_path(:,1);
y_z = points_path(:,2);

pub_traj(traj_setpoints_pub, points_path)

for i=1:2
    T1 = T_goal;
    T1(1:3,4) = [T_goal(1,4) y_y(i) y_z(i)]';
    T2 = T_goal;
    T2(1:3,4) = [T_goal(1,4) y_y(i+1) y_z(i+1)]';
    
    dist = sqrt((y_y(i)-y_y(i+1))^2+(y_z(i)-y_z(i+1))^2);
    
    [Ti_traj, Tdi_traj, omegai_des, pddi_des, omegadi_des] = traj_generation(T1, T2, Ti, 1, 100);
    T_traj = cat(3,T_traj,Ti_traj);
    Td_traj = cat(3,Td_traj,Tdi_traj);
    omega_des = cat(2,omega_des,omegai_des);
    pdd_des = cat(2,pdd_des,pddi_des);
    omegad_des = cat(2,omegad_des,omegadi_des);
end

Tfinal = [ eul2rotm([1.571 -0.025, +3.142]),[T_traj(1,4,end) -0.43 -0.48]';0 0 0 1];

[Tf_traj, Tdf_traj, omegaf_des, pddf_des, omegadf_des] = traj_generation(T_traj(:,:,end), Tfinal, Ti, 1.5, 150);

T_traj = cat(3,T_traj,Tf_traj);
Td_traj = cat(3,Td_traj,Tdf_traj);
omega_des = cat(2,omega_des,omegaf_des);
pdd_des = cat(2,pdd_des,pddf_des);
omegad_des = cat(2,omegad_des,omegadf_des);

index = 750 -100;

for i=1:index
    R_des(:,:,i) = T_traj(1:3,1:3,i);
    p_des(:,i) = T_traj(1:3,4,i);
    
    pd_des(:,i) = Td_traj(1:3,4,i);
end

for i=1:50+100
    R_des(:,:,end+1) = Tfinal(1:3,1:3);
    p_des(:,end+1) = Tfinal(1:3,4);
    
    pd_des(:,end+1) = pd_des(:,index);
    T_traj(:,:,end+1) = Tfinal;
    Td_traj(:,:,end+1) = Td_traj(:,:,index);
    omega_des(:,end+1) = omega_des(:,index);
    pdd_des(:,end+1) = pdd_des(:,index);
    omegad_des(:,end+1) = omegad_des(:,index);
end
i=0;
while(i<10)
    pub_traj(traj_setpoints_pub, points_path);
    i=i+1;
end
rosshutdown

%% ----------------------------   PLOTS   ---------------------------------
t_vec = linspace(0,8,800);
% Plotting the workspace 
figure()

% plot(cc(1,1),cc(1,2),'+k')
hold on
% plot(cc(2,1),cc(2,2),'+k')
% plot(cc(3,1),cc(3,2),'+k')

%-----Workspace-----
A = [cc(1,1)-box_len/2,cc(1,2)-box_len/2];
rectangle('Position',[A, box_len, box_len],'linewidth', 1.5)

%-----Obstacles-----
C = [cc(2,1)-r1,cc(2,2)-r1];
rectangle('Position',[C, r1*2, r1*2], 'Curvature', [1,1],'linewidth', 1.5)
D = [cc(3,1)-r2,cc(3,2)-r2];
rectangle('Position',[D, r2*2, r2*2], 'Curvature', [1,1],'linewidth', 1.5)

%------Collison free area------
 C_aug = C - robot_radius - 0.01;
rectangle('Position',[C_aug, r1_aug*2, r1_aug*2], 'Curvature', [1,1],'LineStyle', '-.','EdgeColor','k','linewidth', 1.5)
 D_aug = D - robot_radius - 0.01;
rectangle('Position',[D_aug, r2_aug*2, r2_aug*2], 'Curvature', [1,1],'LineStyle', '-.','EdgeColor','k','linewidth', 1.5)
axis equal
% axis ([-4 4 -4 4])

plot(q0(1,1),q0(1,2),"*k",'linewidth', 1.5)
% plot(qg(1,1),qg(1,2),"*g")
plot(Tfinal(2,4),Tfinal(3,4),'*k','linewidth', 1.5)
% plot(traj_planned(:,1),traj_planned(:,2),'k')  
plot(squeeze(T_traj(2,4,301:end)),squeeze(T_traj(3,4,301:end)),'k','linewidth', 1.5)
set(gca,'fontsize',18)
%

% Trajectory in Cartesian Space
figure
subplot(3,1,1)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,p_des(1,:),'k','linewidth', 2)
hold on
plot(t_vec,p_des(2,:),'--k','linewidth', 2)
plot(t_vec,p_des(3,:),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% title('Traiettoria lineare')
hold on 
% legend('x','y','z','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$p_{des}$ [m]','interpreter','latex')
subplot(3,1,2)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,pd_des(1,:),'k','linewidth', 2)
hold on
plot(t_vec,pd_des(2,:),'--k','linewidth', 2)
plot(t_vec,pd_des(3,:),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% legend('xd','yd','zd','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$\dot{p}_{des}$ [m/s]','interpreter','latex')
subplot(3,1,3)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,pdd_des(1,:),'k','linewidth', 2)
hold on
plot(t_vec,pdd_des(2,:),'--k','linewidth', 2)
plot(t_vec,pdd_des(3,:),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% legend('xdd','ydd','zdd','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$\ddot{p}_{des}$  [m/$s^2$]','interpreter','latex')


ang_path = rotm2eul(R_des);
figure
subplot(3,1,1)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,ang_path(:,1),'k','linewidth', 2)
hold on
plot(t_vec,ang_path(:,2),'--k','linewidth', 2)
plot(t_vec,ang_path(:,3),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% title('Traiettoria angolare')
hold on 
% legend('yaw','pitch','roll','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$\theta_{des}$ [rad]','interpreter','latex')
subplot(3,1,2)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,omega_des(1,:),'k','linewidth', 2)
hold on
plot(t_vec,omega_des(2,:),'--k','linewidth', 2)
plot(t_vec,omega_des(3,:),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% legend('\omega_x','\omega_y','\omega_z','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$\omega_{des}$ [rad/s]','interpreter','latex')
subplot(3,1,3)
% set(gca, 'ColorOrder', [0 0 0; 0 0 1; 0.8 0 0],'NextPlot', 'replacechildren');
plot(t_vec,omegad_des(1,:),'k','linewidth', 2)
hold on
plot(t_vec,omegad_des(2,:),'--k','linewidth', 2)
plot(t_vec,omegad_des(3,:),':k','linewidth', 2)
set(gca,'fontsize',18)
grid on
% legend('\omega_x d','\omega_y d','\omega_z d','Location', 'bestoutside')
xlabel('$t$ [s]','interpreter','latex')
ylabel('$\dot{\omega}_{des}$ [rad/$s^2$]','interpreter','latex')
