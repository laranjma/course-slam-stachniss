function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q = 0.1 * eye(2);

% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  rx = robot(1);
  ry = robot(2);
  rt = robot(3);

  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)
      % TODO: Initialize its position based on the measurement and the current robot pose:
      alpha = normalize_angle(z(j).bearing + rt);
      particles(i).landmarks(l).mu(1) = rx + z(j).range * cos(alpha);
      particles(i).landmarks(l).mu(2) = ry + z(j).range * sin(alpha);

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
      particles(i).landmarks(l).sigma = pinv(H) * Q * (pinv(H)');
      
      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else
      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      Sg = particles(i).landmarks(l).sigma;
      Sm = H * Sg * (H') + Q;      
      % TODO: calculate the Kalman gain
      K = Sg * H' * inv(Sm);
      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
      dz = [z(j).range - expectedZ(1); normalize_angle(z(j).bearing - expectedZ(2))];
      % TODO: update the mean and covariance of the EKF for this landmark
      particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K * dz;
      particles(i).landmarks(l).sigma = (eye(2) - K * H) * particles(i).landmarks(l).sigma;
      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      particles(i).weight = particles(i).weight / sqrt(det(2*pi*Sm)) * exp(-0.5 * dz' * inv(Sm) * dz);
    end

  end % measurement loop
end % particle loop

end
