function M = inertia_matrix(theta, parameters)

    % This function computes the inertia matrix of the manipulator by calling
    % N_JOINTS calls of the inverse dynamic model
      
    N_JOINTS = parameters.N_JOINTS;
    M = zeros(N_JOINTS);

    for i=1:N_JOINTS
        fake_acc = zeros(N_JOINTS,1);
        fake_acc(i) = 1;
        M(:,i) = recursive_invdyn_fun(theta, zeros(21,1), fake_acc, [0 0 0], zeros(6,1), parameters);
    end

end