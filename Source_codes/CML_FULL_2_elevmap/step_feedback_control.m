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
%bicycle model feedback control motion

for idx = 1:simu.N
	if simu.i <= simu.delayN + 1
		%need to update for using particle update and dead reckoning during the delay time
		fc.x0 = [agent(idx).px(simu.i-1) agent(idx).py(simu.i-1) agent(idx).theta(simu.i-1)];
	else
		%input will be at simu.i-1 step, pf update + delta dead reckoning during delay time
		fc.x = mn(idx).px(simu.i - 1 -simu.delayN) + agent(idx).px(simu.i-1) -agent(idx).px(simu.i - 1 -simu.delayN);
		fc.y = mn(idx).py(simu.i - 1 -simu.delayN) + agent(idx).py(simu.i-1) -agent(idx).py(simu.i - 1 -simu.delayN);
		fc.theta = mn(idx).theta(simu.i - 1 -simu.delayN) + agent(idx).theta(simu.i-1) -agent(idx).theta(simu.i - 1 -simu.delayN);
% 
		fc.x0 = [fc.x fc.y fc.theta];

		% fc.x0 = [agent(idx).tpx(simu.i-1) agent(idx).tpy(simu.i-1) agent(idx).ttheta(simu.i-1)];	%px,py,theta from turth
	end
	fc.L = agent(idx).line;
	fc.v = agent(idx).gvel(simu.i);

	%function [x y theta] = fn_feedback_control(x0, L, v, kd, kh, Ts, dt, l, max_vel, min_vel, max_steering, min_steering)
	[fc.posex fc.posey fc.posetheta] = fn_feedback_control(fc.x0, fc.L, fc.v, simu.kd,simu.kh,simu.Ts,simu.dt,simu.l,simu.max_vel,simu.min_vel,simu.max_steering,simu.min_steering);
	%be careful about the difference between the local coordinate and global coordiante
	agent(idx).ttheta(simu.i) = agent(idx).ttheta(simu.i-1) + (fc.posetheta-fc.x0(3));	%truth theta from feedback control (rad)
	agent(idx).tpx(simu.i) = agent(idx).tpx(simu.i-1) + sqrt((fc.posex-fc.x0(1))^2  + (fc.posey-fc.x0(2))^2) * cos(agent(idx).ttheta(simu.i));	%truth x from feedback control
	agent(idx).tpy(simu.i) = agent(idx).tpy(simu.i-1) + sqrt((fc.posex-fc.x0(1))^2  + (fc.posey-fc.x0(2))^2) * sin(agent(idx).ttheta(simu.i));	%truth y from feedback control

	%truth velocity from feedback control
	agent(idx).tvel(simu.i) = sqrt((fc.posex-fc.x0(1))^2  + (fc.posey-fc.x0(2))^2) / simu.Ts;
	agent(idx).tdtheta(simu.i) = (fc.posetheta-fc.x0(3)) / simu.Ts;

	%odometry measurements (noisy)
	agent(idx).vel(simu.i) = normrnd(agent(idx).tvel(simu.i),simu.sigmaVelocity) + agent(idx).biasVelocity;
    agent(idx).dtheta(simu.i) = normrnd(agent(idx).tdtheta(simu.i),simu.sigmaYawRate) + agent(idx).biasYawRate;
end

%generate ranging measurements (noisy and truth)
for idx1=1:simu.N 
	%relative position
	agent(idx1).trx(simu.i) = agent(idx1).tpx(simu.i) - agent(1).tpx(simu.i);                       
    agent(idx1).try(simu.i) = agent(idx1).tpy(simu.i) - agent(1).tpy(simu.i);
    %ranging
    for idx2=1:simu.N
        agent(idx1).trange(simu.i,idx2) = fn_dist(agent(idx1).tpx(simu.i),agent(idx1).tpy(simu.i),agent(idx2).tpx(simu.i),agent(idx2).tpy(simu.i));
        % agent(idx1).range(simu.i,idx2) = normrnd(agent(idx1).trange(simu.i,idx2),simu.sigmaRangeRatio*agent(idx1).trange(simu.i,idx2))+simu.RangeDelayError;
        agent(idx1).range(simu.i,idx2) = normrnd(agent(idx1).trange(simu.i,idx2),simu.sigmaRange)+simu.RangeDelayError;
        agent(idx2).range(simu.i,idx1) = agent(idx1).range(simu.i,idx2);
    end
end

%clean up
clear idx idx1 idx2 fc