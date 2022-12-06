function [Te, Te_dot, varargout] = traj_generation(Ts, Tg, Ti, Tf, N)

    % This function computes the trajectory in the cartesian space starting
    % from intial and final configurations expressed through theri
    % transformation matrices.
    % The inputs are:
    % - Ts = Transformation matrix of the initial ee pose
    % - Tg = Transformation matrix of the final ee pose
    % - Ti = Initial instant of time
    % - Tf = Final instant of time
    % - N = Number of points for the desired trajectory
    % The outputs are:
    % - Te = vector of transformation matrices through the whole trajectory
    % - Te_d = Time derivative of the previous vector
    % - varargout{1} = The vector of angular velocities of the ee
    % - varargout{2} = The vector of linear accelerations through the whole
    %                  trajectory
    % - varargout{3} = The vector of angular accelerations through the whole
    %                  trajectory

    format long
    
    % Starting and goal position and orientation
    Rs = Ts(1:3,1:3);
    ps = Ts(1:3,4);
    
    Rg = Tg(1:3,1:3);
    pg = Tg(1:3,4);
    
    % Time interval
    T = Tf - Ti;
    
    % Coefficients of the fifth order polinomial
    a1 = 10/T^3; 
    a2 = -15/T^4;
    a3 = 6/T^5;
    
    % Displacements parameters among Rs and Rg
    Rsg = Rs'*Rg;
    % These two equations can be used instead of rotm2axang
    %     th_f = acos((Rsg(1,1) + Rsg(2,2) + Rsg(3,3) -1)/2);
    %     r = 1/(2*sin(th_f)) * [Rsg(3,2) - Rsg(2,3); Rsg(1,3) - Rsg(3,1); Rsg(2,1) - Rsg(1,2)];
    r = rotm2axang(Rsg)';
    th_f = r(4);
    r(4) = [];
    
    
    for i=1:N
        t = T/(N-1)*(i-1);
        
        % Arc length for the rectilinear path
        s = a1*t^3 + a2*t^4 + a3*t^5;
        sd = 3*a1*t^2 + 4*a2*t^3 + 5*a3*t^4;
        sdd = 6*a1*t + 12*a2*t^2 + 20*a3*t^3;
        
        % Rectilinear path
        p = ps + s*(pg-ps);
        pd = sd*(pg-ps);
        pdd(:,i) = sdd*(pg-ps);
        
        % Angle between Rs and Rg
        th = a1*t^3*th_f + a2*t^4*th_f + a3*t^5*th_f;
        thd = 3*a1*t^2*th_f + 4*a2*t^3*th_f + 5*a3*t^4*th_f;
        thdd = 6*a1*t*th_f + 12*a2*t^2*th_f + 20*a3*t^3*th_f;
        
        % Angular velocity and acceleration of the "middle frame" R^i
        omega_i = thd*r;
        omegad_i = thdd*r;
        
        % Angular path
        Re = Rs*axang2rotm([r; th]');
        omega_e(:,i) = Rs*omega_i;
        omegad_e(:,i) = Rs*omegad_i;
        
        % Te_dot and Te
        Re_dot = skew_f(omega_e(:,i))*Re;
        
        Te_dot(:,:,i) = [Re_dot pd; 0 0 0 0];
        Te(:,:,i) = [Re p; 0 0 0 1];
    end
    
    varargout{1} = omega_e;     % First optional output is the desired angular velocity
    varargout{2} = pdd;         % Second optional output is the desired linear acceleration
    varargout{3} = omegad_e;    % Third optional output is the desired angular acceleration


end