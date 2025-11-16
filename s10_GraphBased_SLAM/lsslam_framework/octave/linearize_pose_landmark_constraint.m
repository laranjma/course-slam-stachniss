% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  X = v2t(x);
  R = X(1:2,1:2);
  e = (R')*(l - x(1:2)) - z;
  
  xl = l(1);
  yl = l(2);
  xi = x(1);
  yi = x(2);
  thetai = x(3);
  A = [-cos(thetai),-sin(thetai), -sin(thetai)*(xl-xi)+cos(thetai)*(yl-yi);
       sin(thetai), -cos(thetai),-cos(thetai)*(xl-xi)-sin(thetai)*(yl-yi)];
       
  B = -A(:,1:2);

end;
