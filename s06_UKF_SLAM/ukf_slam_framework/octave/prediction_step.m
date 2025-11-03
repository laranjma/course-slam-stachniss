function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

disp('.........prediction step.........')

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
m = size(sigma_points,2) % 2*n+1;
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
new_thetas = normalize_angle(sigma_points(3,:) + u.r1);
deltas = [u.t * cos(new_thetas);
          u.t * sin(new_thetas);
          (u.r1 + u.r2) * ones(1,m)]
sigma_points(1:3,:) = sigma_points(1:3,:) + deltas;
sigma_points(3,:) = normalize_angle(sigma_points(3,:));

% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;



% TODO: Recover sigma. Again, normalize the angular difference


% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma


end
