%{
/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) <2018>, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
%}
%%
plots.markersize=5;
plots.linewidth=1;
plots.colors(1,:) = [0,0,0]; %black
plots.colors(2,:) = [1,0,0]; %red
plots.colors(3,:) = [0,0,1]; %blue
plots.colors(4,:) = [0,1,0]; %green
plots.colors(5,:) = [.5,.5,0]; %yellow
plots.colors(6,:) = [0,1,1]; %cyan
plots.colors(7,:) = [1,0,1]; %magenta
plots.colors(8,:) = [1,0.5,0]; %orange
plots.colors(9,:) = [0.5,0,1]; %purple
plots.colors(10,:) = [0,0.5,0.25]; %pale blue
%% POSITION ERROR
figure;
clf
subplot(311)
plots.time = simu.Ts*1:length(errors.mn(1).px);
for i=1:simu.N
% for i=1:1
    plots.err_posx_dr(i)=plot(plots.time,errors.dr(i).px,'LineStyle',':','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
%     plots.err_posx_cl(i)=plot(plots.time,errors.cl(i).px,'LineStyle','--','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on;
    plots.err_posx_mn(i)=plot(plots.time,errors.mn(i).px,'LineStyle','-.','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
%     plots.cov_posx_cl(i)=plot(plots.time,sqrt(cl(i).covx),'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
end
ylabel('Error (Meters)');
xlabel('Time Step (Seconds)');
grid on;
title('X Position Error (Global Frame)');
legend([plots.err_posx_dr(1),plots.err_posx_mn(1)],'DR','PF');
% legend([plots.err_posx_dr(1),plots.err_posx_cl(1),plots.cov_posx_cl(1)],'DR','EKF','\sigma');

subplot(312)
plots.time = simu.Ts*1:length(errors.mn(1).py);
for i=1:simu.N
% for i=1:1
    plots.err_posy_dr(i)=plot(plots.time,errors.dr(i).py,'LineStyle',':','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
%     plots.err_posy_cl(i)=plot(plots.time,errors.cl(i).py,'LineStyle','--','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on;
    plots.err_posy_mn(i)=plot(plots.time,errors.mn(i).py,'LineStyle','-.','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
%     plots.cov_posy_cl(i)=plot(plots.time,sqrt(cl(i).covy),'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
end
ylabel('Error (Meters)');
xlabel('Time Step (Seconds)');
grid on;
title('Y Position Error (Global Frame)');
legend([plots.err_posy_dr(1),plots.err_posy_mn(1)],'DR','PF');
% legend([plots.err_posy_dr(1),plots.err_posy_cl(1),plots.cov_posy_cl(1)],'DR','EKF','\sigma');

%% HEADING ERROR
subplot(313)
plots.time = simu.Ts*1:length(errors.mn(1).p);
for i=1:simu.N
% for i=1:1
    plots.err_heading_dr(i)=plot(plots.time,180/pi*errors.dr(i).theta,'LineStyle',':','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
%     plots.err_heading_cl(i)=plot(plots.time,180/pi*errors.cl(i).theta,'LineStyle','--','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
    plots.err_heading_mn(i)=plot(plots.time,180/pi*errors.mn(i).theta,'LineStyle','-.','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
    % plots.cov_heading_cl(i)=plot(plots.time,sqrt(cl(i).covtheta),'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on;
    % plots.cov_heading_cl(i)=plot(plots.time,-sqrt(cl(i).covtheta),'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
end
ylabel('Error (Degrees)');
xlabel('Time Step (Seconds)');
grid on;
title('Heading Error (Global Frame)');
legend([plots.err_heading_dr(1),plots.err_heading_mn(1)],'DR','PF');
% legend([plots.err_heading_dr(1),plots.err_heading_cl(1),plots.cov_heading_cl(i)],'DR','EKF','\sigma');

%% MAGNETIC RESIDUAL
% figure;
% plots.time = simu.Ts*1:length(mn(1).resid);
% % for i=1:simu.N
% for i=1:1
%     plots.resid(i)=plot(plots.time,mn(i).resid,'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
% end
% ylabel('Resid (nT)');
% xlabel('Time Step (Seconds)');
% grid on;
% title('Residual of Particle filter');
% %% CLEAN UP
clear markersize linewidth i