function Jac = snake_space_jacobian(theta, index, parameters)

    [N_JOINTS, ~, x_distances, y_distances, z_distances, ~, ~, ~, ~, ~, ~] = param_assignments(parameters); 

    % This function computes the Space Jacobian for the robot.
    % PLEASE NOTE: the first 3 rows are referred at the angular part and the
    % last 3 rows to the linear one.
    % - theta is the vector of joint angles
    % - index is 0 if we want the full Jacobian, 1 if we want che reduced
    %   one
    % - parameters is the set of kinematic and dynamic parameters for the
    %   robot as read from the URDF file.
    
    format long

    % Rotation axis of the previous (i-1) frame for each joint
    rot_vec = [0 3 1 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3];
    % Rotation axis of the actual (i) frame for each joint
    curr_rot = [3 1 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 2];

    base_disp = [0 0 0]';

    q = [];
    R_curr = eye(3);
    omega_vec = [];
    J = zeros(6,21);

    for i=1:N_JOINTS

        switch rot_vec(i)
            case 0
                R_temp = eye(3);
            case 1
                R_temp = rotx(rad2deg(theta(i-1)));
            case 2
                R_temp = roty(rad2deg(theta(i-1)));
            case 3
                R_temp = rotz(rad2deg(theta(i-1)));
        end

        R_curr = R_curr * R_temp;

        q_disp = [x_distances(i) y_distances(i) z_distances(i)];

        q = base_disp + R_curr*q_disp';
        base_disp = q;

        switch curr_rot(i)
            case 1
                omega_vec = [1 0 0]';
            case 2
                omega_vec = [0 1 0]';
            case 3
                omega_vec = [0 0 1]';
        end
        omega = R_curr * omega_vec;

        v = cross(-omega, q);

        J(:,i) = [omega; v];
    end
    
   
    % Chosing which jacobian must be sent as output
    if index==0
        Jac = J;
    elseif index==1
        % Removing linearly dependent columns from the jacobian
        Js = J(:,1);
        for i=2:length(J(1,:))
            if rank(Js)<rank([Js J(:,i)])
                Js = [Js J(:,i)];
            end
        end
        Jac = Js;
    else
        error('Index value not correct.')
    end
    
end
    
    
    
