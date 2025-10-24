function [w_z] = landmark_loc(p, z)
% Computes the localisation of a landmark in the world frame,
% from the robot pose w_p_r and a given observation z.

% Make tools available
addpath('tools');

% Homogeneous transform
r_z = [z(1); z(2); 1]; % observation in robot frame
w_M_r = v2t(p);        % robot pose in world frame
w_z = w_M_r * r_z;     % observation in world frame

end