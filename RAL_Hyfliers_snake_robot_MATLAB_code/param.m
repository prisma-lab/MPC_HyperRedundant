%---------------DEBUG (Comment load and uncomment importrobot)--------------

% snake_robot = importrobot('prisma_snake.urdf');
load('snake_robot_new.mat')

format long
N_JOINTS = 21;

%% PARAMETERS

% Vector of Relative Transformation matrix from i-1-th joint to i-th
Transf_mat = [];
for i=1:N_JOINTS+3
    Transf_mat(:,:,i) = snake_robot.Bodies{1,6+i}.Joint.JointToParentTransform;
end

Transf_mat(:,:,1) = Transf_mat(:,:,1) * Transf_mat(:,:,2) * Transf_mat(:,:,3);
Transf_mat(:,:,2) = [];
Transf_mat(:,:,2) = [];
Transf_mat(:,:,3) = Transf_mat(:,:,3) * Transf_mat(:,:,4);
Transf_mat(:,:,4) = [];

% Storing kineamtic and dynamic parameters
axis = [];                  % Rotational axis of the current joint with respect to its frame
mass = [];                  % Mass vector
Inertia = [];               % Inertia tensors
inertial_dist = [];         % Inertia displacements (joint frame - Center of Mass)
Limits = [];                % Limits of the joints

for i=1:N_JOINTS+1

    axis(:,i) = snake_robot.Bodies{1, 8+i}.Joint.JointAxis;
    mass(i) = snake_robot.Bodies{1,8+i}.Mass;
    in_vec = snake_robot.Bodies{1,8+i}.Inertia;
    Inertia(:,:,i) = [in_vec(1) 0 0; 0 in_vec(2) 0; 0 0 in_vec(3)];
    inertial_dist(i,:) = snake_robot.Bodies{1,8+i}.CenterOfMass;
    Limits(i,:) = snake_robot.Bodies{1,8+i}.Joint.PositionLimits;

end

axis(:,3) = [];

mass(3) = [];
mass = mass';

Inertia(:,:,3) = [];
Inertia(:,:,1) = zeros(3,3);
clear in_vec

inertial_dist(3,:) = [];

Limits(3,:) = [];
qm = Limits(:,1);
qM = Limits(:,2);
qM(1) = 6*pi;
qm(1) = -6*pi;
clear Limits

% Distances between each joint and the parent
x_distances = [-0.0101 0 -0.180045];
z_distances = [0.155 0 0.0224478];
for i=1:17
    x_distances = [x_distances -0.0525];
    z_distances = [z_distances 0];
end
x_distances = [x_distances -0.033];
z_distances = [z_distances 0.0239999];

y_distances = zeros(1, length(x_distances));
y_distances(1) = 0.00772822; 

% S vector (screw axis referred to the base frame)
% v (theta=0)
v = [];
S = [];
omega = [];
T_rest = eye(4);

for i=1:N_JOINTS
   T_rest(:,:,i) = T_rest(:,:,end)*Transf_mat(:,:,i); 
   omega(i,:) =  round(T_rest(1:3,1:3,i)*axis(:,i));
   
    X = 0;
    Y = 0;
    Z = 0;

    for j=1:i
        X = X + x_distances(j);
        Y = Y + y_distances(j);
        Z = Z + z_distances(j);
    end

    q = [X Y Z];
    v = [v; cross(-omega(i,:),q)];
    
    S(i,:) = [omega(i,:) v(i,:)];
end
clear X Y Z q

% Rotation axis and relative angles to reach joint i from i-1
rot_mat = [1 2;
           1 2;
           1 2;];

rot_angle = [pi/2 pi;
             -pi/2 pi/2;
             pi/2 pi;];

for i=3:18
    rot_mat = vertcat(rot_mat, rot_mat(end,:));
    rot_angle = vertcat(rot_angle, rot_angle(end,:));
end

rot_mat = [rot_mat;
             1 2;
             1 2;];
rot_angle = [rot_angle;
             pi/2 pi/2;
             pi/2 pi/2;];
         
         
M_prev = eye(4);
A = zeros(6,21);
G = zeros(6,6,21);
M_save = zeros(4,4,21);

for i=1:N_JOINTS

    % Computation of T(i-1,i)
    M = M_f(rot_mat(i,:), rot_angle(i,:), x_distances, y_distances, z_distances, i);  % from 0 to i
    A_i = Ad_f(inv(M))*S(i,:)';
    G_i = G_f(mass(i),Inertia(:,:,i),inertial_dist(i,:));

    M_mutual = inv(M_prev)*M;       % from i-1 to i
    M_prev = M;  

    % Saving previuos quantities in matrices
    A(:,i) = A_i;
    G(:,:,i) = G_i;
    M_save(:,:,i) = M_mutual;

end         
         
         
                 
% Parameters struct
parameters.N_JOINTS = N_JOINTS;
parameters.T_rest = T_rest;
parameters.x_distances = x_distances;
parameters.y_distances = y_distances;
parameters.z_distances = z_distances;
parameters.S = S;
parameters.mass = mass;
parameters.Inertia = Inertia;
parameters.inertial_dist = inertial_dist;
parameters.rot_mat = rot_mat;
parameters.rot_angle = rot_angle;
parameters.A = A;
parameters.G = G;
parameters.M = M_save;

clear i j
