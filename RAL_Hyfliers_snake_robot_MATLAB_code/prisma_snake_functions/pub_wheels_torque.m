function pub_wheels_torque(publisher, message)

    % This function is used to publish the message in the topic specified by publisher
    
    N_wheels = 4;
    J_msg = zeros(N_wheels,1);
    
    % Tw1 dx e Tw2 sx
    J_msg(1) = -message(2)/2;
    J_msg(2) = -message(3)/2;
    J_msg(3) = -message(2)/2;
    J_msg(4) = -message(3)/2;
    
    J_msg_std = rosmessage('std_msgs/Float64');
    for j=1:N_wheels
        J_msg_std.Data = J_msg(j);
        send(publisher(j), J_msg_std);
    end

end