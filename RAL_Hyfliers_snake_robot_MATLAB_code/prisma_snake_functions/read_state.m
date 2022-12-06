function [theta, theta_d, varargout] = read_state(model_state_latest_mex)
    
    % This function is used to read the state of the model in the scene
    % reading the messages published by gazebo_msgs/ModelState. It is used
    % to obtain the pose of the robot in the scene and the angular position
    % and velocity of itself around the pipe.

    model_state = rosmessage('gazebo_msgs/ModelState');

    model_state = model_state_latest_mex;
    snake_position = [model_state.Pose(end).Position.X model_state.Pose(end).Position.Y model_state.Pose(end).Position.Z]; 
    snake_orientation_t = [model_state.Pose(end).Orientation.W model_state.Pose(end).Orientation.X model_state.Pose(end).Orientation.Y model_state.Pose(end).Orientation.Z]; 
    snake_orientation = quat2eul(snake_orientation_t);
    theta = snake_orientation(3);
    theta_d = (model_state.Twist(end).Angular.X);
    
    R = quat2rotm(snake_orientation_t);
    T = [R snake_position'; 0 0 0 1];
    
    if nargout>=3
        varargout{1} = T;
    end
end
      