function [N_JOINTS, T_rest, x_distances, y_distances, z_distances, S, mass, Inertia, inertial_dist, rot_mat, rot_angle] = param_assignments(parameters) 

    N_JOINTS = parameters.N_JOINTS;
    T_rest = parameters.T_rest;
    x_distances = parameters.x_distances;
    y_distances = parameters.y_distances;
    z_distances = parameters.z_distances;
    S = parameters.S;
    mass = parameters.mass;
    Inertia = parameters.Inertia;
    inertial_dist = parameters.inertial_dist;
    rot_mat = parameters.rot_mat;
    rot_angle = parameters.rot_angle;

end