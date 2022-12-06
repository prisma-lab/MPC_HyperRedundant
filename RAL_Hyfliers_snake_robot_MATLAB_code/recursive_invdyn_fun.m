function tau = recursive_invdyn_fun(theta, thetad, thetadd, gravity, F_ee, parameters)

    % This function computes the inverse dynamic for the robot with the
    % Newton-Euler recursive Algorithm.
    % - theta is the column vector of joint angles
    % - thetad is the column vector of joint velocities
    % - thetadd is the column vector of joint accelerations
    % - gravity is a 1x3 vector of gravity forces applied on the end-effector
    % - F_ee id the 6x1 vector of external forces applied on the end-effector
    % - parameters is the struct generated in the param.m script containing
    %   all the quantities needed to compute the dynamic model.
    % CON LE MODIFICHE FATTE SOLO NELLA DINAMICA ABBIAMO RECUPERATO 30
    % secondi 

    [N_JOINTS, ~, x_distances, y_distances, z_distances, S, mass, Inertia, inertial_dist, rot_mat, rot_angle] = param_assignments(parameters); 
      
    V_prev = [0 0 0 0 0 0]';            % Initial conditions for V
    Vd_prev = [0 0 0 -gravity]';        % Initial conditions for Vd
    
    V = zeros(6,21);
    Vd = zeros(6,21);

    T = zeros(4,4,22);
    A = parameters.A;
    G = parameters.G;
    M_save = parameters.M;
    
    F = zeros(6,21);
    F_prev = F_ee;
    
    tau = zeros(1,21);

    for i=1:N_JOINTS
            
        M_mutual = M_save(:,:,i);
        A_i = A(:,i);
        
        A_bracket = [skew_f(A_i(1:3)) A_i(4:6); 0 0 0 0];
        T_mutual = expm(-(A_bracket)*theta(i))/(M_mutual);     % from i to i-1

        % Saving previuos quantities in matrices
        T(:,:,i) = T_mutual;

        % Computation of V_i and Vd_i saving them in vactors
        V(:,i) = Ad_f(T_mutual)*V_prev + A_i*thetad(i);
        V_prev = V(:,i);
        
        Vd(:,i) = Ad_f(T_mutual)*Vd_prev + ad_f_(V_prev)*A_i*thetad(i) + A_i*thetadd(i);
        Vd_prev = Vd(:,i);
    end
    
    T(:,:,end) = eye(4);  % Adding one element in order to use the following for from N to 1
                            % (Probe_joint and end_effector frames are coincident)

    for i=N_JOINTS:-1:1
        F(:,i) = Ad_f(T(:,:,i+1))'*F_prev + G(:,:,i)*Vd(:,i) - ad_f_(V(:,i))'*(G(:,:,i)*V(:,i));
        F_prev = F(:,i);
        
        tau(i) =  F_prev'*A(:,i);
    end
      
end