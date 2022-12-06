function G = G_f(m,I,inertial_disp)

    % This function computes the G_i matrix for the i-th link.
    % - m is the mass of the link in the center of mass
    % - I is the inertia tensor referred to the center of mass
    % - inertial_disp is the dispacement between the link frame and the
    %   inertial frame
    % We consider null rotation between these two frames.
    
    Gc = [I zeros(3,3);
          zeros(3,3) m*eye(3)];
     
    T = eye(4);
    T(1:3,4) = inertial_disp;
    
    G = Ad_f(inv(T))'*Gc*Ad_f(inv(T));

end