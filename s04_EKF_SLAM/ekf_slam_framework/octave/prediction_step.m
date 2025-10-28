function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

theta = normalize_angle(mu(3));
delta = [u.t * cos(theta + u.r1);
         u.t * sin(theta + u.r1);
         u.r1 + u.r2];
new_mu = mu + delta;
new_mu(3) = normalize_angle(new_mu(3)); 
mu = new_mu;

% TODO: Compute the 3x3 Jacobian Gx of the motion model

Gx = eye(3,3) + [0, 0, -u.t * sin(theta + u.r1)
                 0, 0,  u.t * cos(theta + u.r1)
                 0, 0,  0];

% TODO: Construct the full Jacobian G

% G = [Gx   ZERO
%      ZERO  I  ]
% dim(G) = (2N+3) x (2N+3) == dim(sigma)
G = eye(size(sigma,1)); 
G(1:3,1:3) = Gx;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion

new_sigma = G * sigma * G' + R;
sigma = new_sigma;

end
