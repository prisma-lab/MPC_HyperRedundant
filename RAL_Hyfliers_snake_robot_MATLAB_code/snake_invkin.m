function [theta_d, theta, w, w2, Vb] = snake_invkin(Tsd, Tsd_d, init_conditions, dt, parameters)

    format long

    % This function computes the inverse kinematics for the robot
    % - Tsd is the desired pose(s) for the end effector expressed with a
    %   transformation matrix.
    % - Tsd_d is the time derivative of the transformation matrices of
    %   before.
    % - init_conditions is a vector of configurations for the joints near
    %   to the actual goal.
    % - dt is
    % - parameters is the set of kinematic and dynamic parameters for the
    %   robot as read from the URDF file.
    
    K = eye(6)*150;
    theta(:,1) = init_conditions;
    w = [];
    pos_err = [0 0 0]';
    
    for i=1:length(Tsd(1,1,:))
        Td_T = Tsd_d(:,:,i)*inv(Tsd(:,:,i));
        Vbd = [Td_T(3,2); Td_T(1,3); Td_T(2,1); Td_T(1:3,4)];
        
        Tsb = snake_dirkin(theta(:,i), parameters);
        TTinv_mat = logm(inv(Tsb) * Tsd(:,:,i));                                            %4x4
        TTinv_vec = [TTinv_mat(3,2); TTinv_mat(1,3); TTinv_mat(2,1); TTinv_mat(1:3,4)];     %6x1
        Vb(:,i) = Ad_f(Tsb) * TTinv_vec;
        
        J = snake_space_jacobian(theta(:,i), 0, parameters);
        J_pinv = pinv(J);
        
        w(i) = sqrt(det(J*J'));
        
        w2(i) = 0;
        for j = 1:21
            w2(i) = w2(i) + sin(theta(j,i))^2;
        end
        w2(i) = w2(i) * 0.5;
        
        theta_d(:,i) = J_pinv*(K*Vb(:,i) + Vbd);
        theta(:,i+1) = theta(:,i) + dt * theta_d(:,i);
    end
    
    % Removing first element from theta (initial conditions)
    theta(:,1) = [];
    
end