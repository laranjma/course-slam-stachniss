% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  Z = v2t(z);
  X1 = v2t(x1);
  X2 = v2t(x2);
  E = inv(Z)*(inv(X1)*X2);
  e = t2v(E);
  % A = de/dx1
  A = zeros(3);
  theta_ij = z(3);
  xi = x1(1);
  yi = x1(2);
  theta_i = x1(3);
  xj = x2(1);
  yj = x2(2);
  theta_j = x2(3);
  
  dex_dxi = -cos(theta_ij)*cos(theta_i) + sin(theta_ij)*sin(theta_i);
  dex_dyi = -cos(theta_ij)*sin(theta_i) - sin(theta_ij)*cos(theta_i);
  dex_dthetai = -(xj-xi)*(cos(theta_ij)*sin(theta_i) + sin(theta_ij)*cos(theta_i)) + (yj-yi)*(cos(theta_ij)*cos(theta_i) - sin(theta_ij)*sin(theta_i));
  dey_dxi = sin(theta_ij)*cos(theta_i) + cos(theta_ij)*sin(theta_i);
  dey_dyi = sin(theta_ij)*sin(theta_i) - cos(theta_ij)*cos(theta_i);
  dey_dthetai = (xj-xi)*(sin(theta_ij)*sin(theta_i) - cos(theta_ij)*cos(theta_i)) + (yj-yi)*(-sin(theta_ij)*cos(theta_i) - cos(theta_ij)*sin(theta_i));
  detheta_dxi = 0;
  detheta_dyi = 0;
  detheta_dthetai = -1;
  
  A = [dex_dxi, dex_dyi, dex_dthetai;
       dey_dxi, dey_dyi, dey_dthetai;
       detheta_dxi, detheta_dyi, detheta_dthetai];
       
  dex_dxj = -dex_dxi;
  dex_dyj = -dex_dyi;
  dex_dthetaj = 0;
  dey_dxj = -dey_dxi;
  dey_dyj = -dey_dyi;
  dey_dthetaj = 0;
  detheta_dxj = 0;
  detheta_dyj = 0;
  detheta_dthetaj = 1;
  
  B = [dex_dxj, dex_dyj, dex_dthetaj;
       dey_dxj, dey_dyj, dey_dthetaj;
       detheta_dxj, detheta_dyj, detheta_dthetaj];

end;
