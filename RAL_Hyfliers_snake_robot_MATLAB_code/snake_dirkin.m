function T = snake_dirkin(theta, parameters, varargin)

    % This function computes the Forward Kinematic for the robot.
    % - theta is the N_JOINTSx1 vector of joint angles.
    % - parameters is the set of kinematic and dynamic parameters for the
    %   robot as read from the URDF file.
    % - varargin{1} is an additional input used when we want to compute the
    %   pose of the i-th frame with respect to base frame of the robot 

    format long
    [N_JOINTS, T_rest, ~, ~, ~, S, ~, ~, ~, ~, ~] = param_assignments(parameters);   
    
    if(nargin >= 3)
        M = T_rest(:,:,varargin{1}); 
        N_JOINTS = varargin{1};
    else
        M = T_rest(:,:,end);
    end
    
    %Skew-symmetric matrices
    T = eye(4);
    for i=1:N_JOINTS
        S_bracket = [skew_f(S(i,1:3)') S(i,4:6)'; 0 0 0 0];
        T = T*expm(S_bracket*theta(i));
    end
    T = T*M;
    
end





