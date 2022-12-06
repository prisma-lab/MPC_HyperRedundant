function [joint_position, joint_velocity] = read_feedback(joint_state_latest_mex)

    % This function accept as input the last message from the joint state
    % publisher (joint_state.LastMessage) of type 'sensor_msgs/JointState'
    % and gives back the two N_JOINTSx1 vectors of joint position and velocities.
    % - joint_state_latest_mex is a 'sensor_msgs/JointState' variable from
    %   the joint state publisher.
    % - joint_position and joint_velocity are N_JOINTSx1 vectors of joint
    %   positions and velocities
    
    joint_state = rosmessage('sensor_msgs/JointState');

    joint_state = joint_state_latest_mex;
    joint_position_t = joint_state.Position;
    joint_velocity_t = joint_state.Velocity;

    joint_position(1:2) = joint_position_t(19:20);
    joint_position(3) = joint_position_t(10);
    joint_position(4:10) = joint_position_t(12:18);
    joint_position(11:19) = joint_position_t(1:9);
    joint_position(20) = joint_position_t(11);
    joint_position(21) = joint_position_t(25);

    joint_velocity(1:2) = joint_velocity_t(19:20);
    joint_velocity(3) = joint_velocity_t(10);
    joint_velocity(4:10) = joint_velocity_t(12:18);
    joint_velocity(11:19) = joint_velocity_t(1:9);
    joint_velocity(20) = joint_velocity_t(11);
    joint_velocity(21) = joint_velocity_t(25);

    joint_position = joint_position';
    joint_velocity = joint_velocity';

end