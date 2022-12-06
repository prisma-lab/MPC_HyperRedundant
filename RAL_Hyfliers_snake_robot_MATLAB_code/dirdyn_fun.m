function thetadd = dirdyn_fun(theta, thetad, tau, gravity, F_ee, parameters)

    % This function computes the direct dynamic for the robot with the
    % Newton-Euler recursive Algorithm.
    % - theta is the column vector of joint angles
    % - thetad is the column vector of joint velocities
    % - thetadd is the column vector of joint accelerations
    % - gravity is a 1x3 vector of gravity forces applied on the end-effector
    % - F_ee id the 6x1 vector of external forces applied on the end-effector
    % - parameters is the struct generated in the param.m script containing
    %   all the quantities needed to compute the dynamic model.
    
    N_JOINTS = parameters.N_JOINTS;
    thetadd = zeros(21,1);
    M = zeros(N_JOINTS);

    for i=1:N_JOINTS
        fake_acc = zeros(N_JOINTS,1);
        fake_acc(i) = 1;
        M(:,i) = recursive_invdyn_fun(theta, zeros(21,1), fake_acc, [0 0 0], zeros(6,1), parameters);
    end
    C = recursive_invdyn_fun(theta, thetad, zeros(21,1), [0 0 0], zeros(6,1), parameters);
    G = recursive_invdyn_fun(theta, zeros(21,1), zeros(21,1), gravity, zeros(6,1), parameters);
    F = recursive_invdyn_fun(theta, zeros(21,1), zeros(21,1), [0 0 0], F_ee, parameters);
%     h = recursive_invdyn_fun(theta, thetad, zeros(21,1), gravity, zeros(6,1), parameters);
    
    thetadd = M\(tau - C - G - F);
%     thetadd = M\(tau - h - F);
    
end
