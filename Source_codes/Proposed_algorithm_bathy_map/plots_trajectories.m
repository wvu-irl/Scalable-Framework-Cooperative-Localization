%{
/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) <2021>, WVU Interactive Robotics Laboratory
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
plots.colors(11,:) = [0,0.5,1]; %
plots.colors(12,:) = [1,0.5,0]; %
plots.colors(13,:) = [0.5,0.5,1]; %
plots.colors(14,:) = [0,0.25,0.25]; %
plots.colors(15,:) = [0,0.25,0.5]; %
plots.colors(16,:) = [1,0.5,0.5]; %
%% GLOBAL TRAJECTORIES
figure;
clf
subplot(211)
for i=1:simu.N
    plots.pos_t(i)=plot([agent(i).tpx(1:end)],[agent(i).tpy(1:end)],'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
    plots.pos_dr(i)=plot([agent(i).px(1:end)],[agent(i).py(1:end)],'LineStyle',':','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
    plots.pos_mn(i)=plot([mn(i).px(1:end)],[mn(i).py(1:end)],'LineStyle','-.','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on;
end
xlabel('X (meters)');
ylabel('Y (meters)');
grid on;
title('Position (Global Frame)');
legend([plots.pos_t(1),plots.pos_dr(1),plots.pos_mn(1)],'Truth','DR','PF');

%% GLOBAL HEADING
subplot(212)
plots.time = simu.Ts*1:length(mn(1).theta);
for i=1:1
    plots.heading_t(i)=plot(plots.time(1:end),[180/pi*agent(i).ttheta(1:length(plots.time))],'LineStyle','-','Color',plots.colors(i,:),'LineWidth',plots.linewidth); hold on; 
    plots.heading_dr(i)=plot(plots.time(1:end),[180/pi*agent(i).theta(1:length(plots.time))],'LineStyle',':','Color',plots.colors(i,:),'LineWidth',plots.linewidth);
    plots.heading_mn(i)=plot(plots.time(1:end),[180/pi*mn(i).theta(1:length(plots.time))],'LineStyle','-.','Color',plots.colors(i,:),'LineWidth',plots.linewidth);
end
xlabel('Time (Seconds)');
ylabel('Heading (Degrees)');
grid on;
title('Heading (Global Frame)');
legend([plots.heading_t(1),plots.heading_dr(1),plots.heading_mn(1)],'Truth','DR','PF');

%% CLEAN UP
clear markersize linewidth i