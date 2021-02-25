%function P = particle_cov(q, x, mean_part)
%calculate covariance in particle filter
%output P: covariance
%input q: particles' weights;
%	   x: state vector of each particle
%      mean_part: average mean of all particles

function P = particle_cov(q, x, mean_part)
      [nx, N] = size(x);
      P_temp = x - repmat(mean_part,1,N);
      P = repmat(q',nx,1).*P_temp*P_temp'; 
end