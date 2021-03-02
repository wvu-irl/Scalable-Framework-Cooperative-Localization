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
%initialize agents' position, heading, and velocity

%initial group offset, a constant distance R and random direction
offset_y = simu.initoffset*sin(2*pi*rand); % group offset y
offset_x = simu.initoffset*cos(2*pi*rand); % group offset x

%parallel trajectory (sin velocity)
for idx=1:simu.N
    %noise
    agent(idx).sigmaVelocity = simu.sigmaVelocity - (simu.percentNoiseDifference)*simu.sigmaVelocity + 2*rand*(simu.percentNoiseDifference)*simu.sigmaVelocity;
    agent(idx).sigmaYawRate = simu.sigmaYawRate - (simu.percentNoiseDifference)*simu.sigmaYawRate + 2*rand*(simu.percentNoiseDifference)*simu.sigmaYawRate;
    agent(idx).biasVelocity = normrnd(0,simu.biasVelocity);
    agent(idx).biasYawRate = normrnd(0,simu.biasYawRate);
    %truth 
    agent(idx).phase = rand*2*pi;   %for velocity
    agent(idx).frequency = 1/20;    %for velocity
    agent(idx).amplitude = simu.amplitude;   %for velocity
    agent(idx).baseline = 1;    %base line for velocity
    agent(idx).tvel(1) = agent(idx).baseline + agent(idx).amplitude*sin(agent(idx).frequency*1*simu.Ts + agent(idx).phase); %each vehicle's initial velocity is the first step at sin function
    
    agent(idx).tpx(1) = 100 + rand*10;
    agent(idx).tpy(1) = 500 + idx * simu.initdistance;%(idx/simu.N)*simu.initdistance;
    agent(idx).ttheta(1) = 0;   %init heading truth
    agent(idx).tdtheta(1) = 0;  %init yaw rate truth
    agent(idx).tvx(1) = agent(idx).tvel(1)*cos(agent(idx).ttheta(1));   %init velocity in x axis truth
    agent(idx).tvy(1) = agent(idx).tvel(1)*sin(agent(idx).ttheta(1));   %init velocity in y axis truth
    %estimates
    agent(idx).vel(1) = agent(idx).tvel(1);
    agent(idx).vx(1) = agent(idx).tvx(1);
    agent(idx).vy(1) = agent(idx).tvy(1); 
    agent(idx).px(1) = agent(idx).tpx(1)+normrnd(0,simu.initUncertainty)+offset_x;
    agent(idx).py(1) = agent(idx).tpy(1)+normrnd(0,simu.initUncertainty)+offset_y;
    agent(idx).theta(1) = agent(idx).ttheta(1);
    agent(idx).dtheta(1) = agent(idx).tdtheta(1);
    %for feedback control
    agent(idx).line = [0 1 -agent(idx).tpy(1)];  %target trajectory
end

%at each timestep, set target velocity
i=2;
accumulatedTime=0;
simulationTime = simu.simulationTime;
while accumulatedTime < simulationTime
    %generate trajectories (truth)
    for idx1=1:simu.N 
        %target velocity
        agent(idx1).gvel(i) = agent(idx1).baseline + agent(idx1).amplitude*sin(agent(idx1).frequency*(i)*simu.Ts + agent(idx1).phase);  
    end
    i = i + 1;
    accumulatedTime = accumulatedTime + simu.Ts;
end

clear i accumulatedTime simulationTime idx idx1 offset_y offset_x