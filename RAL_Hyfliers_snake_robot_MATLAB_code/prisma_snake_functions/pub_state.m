function pub_state(publisher, message)

    % This function is used to publish the message in the topic specified by publisher
    
    N_JOINTS = 21;
    J_msg = zeros(21,1);
    
    for j=1:N_JOINTS
        J_msg(j) = message(j);
    end

    J_msg_std = rosmessage('std_msgs/Float64');
    for j=1:N_JOINTS
        J_msg_std.Data = J_msg(j);
        send(publisher(j), J_msg_std);
    end

end