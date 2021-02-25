close all
figure();
ha=error_ellipse(pf_result(8).cov{1,1}(1:2,1:2), pf_result(8).state{1,1}(1:2));hold on;
hb=error_ellipse(pf_result(8).cov{1,2}(1:2,1:2), pf_result(8).state{1,2}(1:2));hold on;
% hd=error_ellipse(pf_result(1).cov{1,3}(1:2,1:2), pf_result(1).state{1,3}(1:2));hold on;
% he=error_ellipse(pf_result(3).cov{1,4}(1:2,1:2), pf_result(3).state{1,4}(1:2));hold on;
hc=error_ellipse(pf.ci_output(8).cov(1:2,1:2), pf.ci_output(8).state(1:2));hold on;
grid on