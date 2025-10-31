function [mu, sigma] = recover_gaussian(sigma_points_trans, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible
n = size(sigma_points_trans, 1);
m = size(sigma_points_trans, 2); % m=2*n+1

% TODO: compute mu
mu = sigma_points_trans * w_m';

% TODO: compute sigma
mu_sshape = repmat(mu, 1, m);
delta = sigma_points_trans - mu_sshape;
sigma = zeros(n);
for i = 1:m
    sigma += w_c(i) * (delta(:,i) * delta(:,i)');
end

end
