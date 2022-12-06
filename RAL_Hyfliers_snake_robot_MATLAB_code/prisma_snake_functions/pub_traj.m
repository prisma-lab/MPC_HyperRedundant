function pub_traj(publisher, message)

    % This function is used to publish the message in the topic specified by publisher
     
    J_msg = message(:);
    dimension = size(J_msg);

    J_msg_std = rosmessage('std_msgs/Float64MultiArray');
    J_msg_std.Data = [dimension(1)*dimension(2); J_msg];
     
    send(publisher, J_msg_std);

end