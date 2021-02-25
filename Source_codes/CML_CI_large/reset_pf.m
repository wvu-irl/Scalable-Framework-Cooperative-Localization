% reset particles based on CI estimates.
pf.replace_number = 300;
for i = 1:simu.N
    x_temp = repmat(pf.state_temp(:,i), 1, pf.replace_number) ...
       + sqrt(eye(pf.nx, pf.nx) .* pf.cov_temp(:,:,i)) ...
       *  randn(pf.nx, pf.replace_number);
    pick_index = randi(pf.npf,1,pf.replace_number);
    
    pf.x(:,pick_index,i) = x_temp;
    
%    pf.x(:,:,i) = repmat(pf.state_temp(:,i), 1, pf.npf) ...
%        + sqrt(eye(pf.nx, pf.nx) .* pf.cov_temp(:,:,i)) ...
%        *  randn(pf.nx, pf.npf);
   % weight of particles.
%    pf.w(:,i) = ones(pf.npf, 1);
   
   pf.w(pick_index,i) = max(pf.w(:,i));
end