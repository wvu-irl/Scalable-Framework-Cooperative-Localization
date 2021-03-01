%% covariance intersection
% do CI for each vehicle.
% disp('Initializing CI...');
for i=1:size(groups,2)
    %element inverse and sum
    ci.sum_information = zeros(size(pf_result(1).cov{1}));
    for j=1:size(pf_result(i).state,2)
        ci.cov_inv{j} = inv(pf_result(i).cov{j});
        ci.sum_information = ci.sum_information + ci.cov_inv{j};
    end
    % eq(15)
    ci.sum_except_information = 0;
    for j=1:size(pf_result(i).state,2)
        ci.sum_except_information = ci.sum_except_information + ...
            det(ci.cov_inv{j}) - det(ci.sum_information - ci.cov_inv{j});
    end
    
    for j=1:size(pf_result(i).state,2)
        ci.omega(j) = (det(ci.sum_information) - ...
            det(ci.sum_information- ci.cov_inv{j}) ...
            + det(ci.cov_inv{j})) ...
        / (size(pf_result(i).state,2)*det(ci.sum_information) + ...
        ci.sum_except_information);
    end
    ci.sum_cov_inv = zeros(size(pf_result(1).cov{1}));
    ci.sum_state = zeros(size(pf_result(1).state{1}));
    for j=1:size(pf_result(i).state,2)
        ci.sum_cov_inv = ci.sum_cov_inv + ci.omega(j) * ci.cov_inv{j};
        ci.sum_state = ci.sum_state + ci.omega(j) * ci.cov_inv{j} * pf_result(i).state{j};
    end
    
    pf.ci_output(i).cov = inv(ci.sum_cov_inv);
    pf.ci_output(i).state = pf.ci_output(i).cov * ci.sum_state;
    pf.cov_temp(1:3,1:3,i) = pf.ci_output(i).cov;
    pf.state_temp(1:3,i) = pf.ci_output(i).state;
end
%% reset pf result
clear pf_result ci x;
for i=1:size(groups,2)
   pf_result(i).state{1} = zeros(3,1);
   pf_result(i).cov{1} = zeros(3,3); 
end
