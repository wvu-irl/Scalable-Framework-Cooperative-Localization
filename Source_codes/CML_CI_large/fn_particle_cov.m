%function P = particle_cov(q, x, mean_part)
%calculate covariance in particle filter
%output P: covariance
%input q: particles' weights; n*1
%	   x: state vector of each particle nx*n
%      mean_part: average mean of all particles nx*1

function P = fn_particle_cov(q, particles, mean)
     
      [num_states, num_particles] = size(particles);
      cov = zeros(num_states, num_states);
      
      for i = 1:num_states
          for j = 1:num_states
              varaince = q'.*...
                  (particles(i,:) - repmat(mean(i), 1, num_particles)).*...
                  (particles(j,:) - repmat(mean(j), 1, num_particles));

              cov(i,j) = sum(varaince);
          end
      end
      P = cov;
end