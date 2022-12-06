function M = M_f(rot_vec,theta_rest,x_dist,y_dist,z_dist,index)

    % We are computing the transformation matrix from the fixed frame to
    % i-th frame when the robot is in its rest position.
    % - rot_vec is the vector containing the axis around which the actual
    %   frame rotates (0 no rot, 1->x, 2->y, 3->z).
    % - theta_rest is the vector of the angles related to the previous
    %   rotations.
    % - x_dist, y_dist, z_dist are the distances of the i-th frame from the
    %   (i-1)-th frame.
    % - index is the index of the actual link that is considered in the
    %   computations.
    
    R = eye(3);
    for i=1:length(rot_vec)
        switch rot_vec(i)
            case 0
                R_t = eye(3);
            case 1
                R_t = rotx(rad2deg(theta_rest(i)));
            case 2
                R_t = roty(rad2deg(theta_rest(i)));
            case 3
                R_t = rotz(rad2deg(theta_rest(i)));
        end
        R = R * R_t;
    end
    
    X = sum(x_dist(1:index));
    Y = sum(y_dist(1:index));
    Z = sum(z_dist(1:index));
        
    M = [R [X Y Z]';
         0 0 0 1];            

end