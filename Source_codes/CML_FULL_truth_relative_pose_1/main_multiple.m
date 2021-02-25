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

% Cooperative Localization Simulator Main
% Author: Jared Strader and Chizhao Yang
%%
clear all;
% close all;
clc;
rng(1)
% s = rng;    %save seed for testing later

%Simulation Parameters
simu.N=16;    %number of UAVs
simu.fullCommunication=1;    %1: complete communication 0: pairwise communication

simu.simulationTime=10*60;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep
simu.Ts=0.1;    %dead reckoning update rate (second)
simu.updateTs=0.1;  %ranging measurements update rate and information exchange rate (second)

simu.sigmaVelocity=0.1;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.1*pi/180;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaGamma=0.001*pi/180;    %standard deviation of group rotation error (rad)
% simu.sigmaRange=1;   %standard deviation of ranging measurement error (m)
%the std of ranging measurement changes based on the range, sigmaRange = sigmaRangeRatio * truth_range + delay_error
simu.sigmaRangeRatio=1/10000;   %standard deviation of ranging measurement error ratio (Data comes from GNC paper)
simu.RangeDelayError = 0; %range measurement error caused by measurement delay
simu.sigmaRange=0.1;

simu.percentNoiseDifference = 0.01; %slightly varies sigma per agent
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
simu.initUncertainty=1;    %standard deviation of initial position uncertainty (m)
simu.initoffset = 0;    %standard deviation of initial group offset, for testing robustness of the CML (m)
simu.amplitude = 0.5;     %amplitude of the speed (m/s) ps: base line for velocity is 50 m/s
simu.initdistance = 200;   %distance between each pair of neighbot UAVs' initial positions


%For particle filter parameters
simu.sigmaMagnetic=1; %standard deviation of magnetic measurement error (nT)
simu.magneticMap = 1;    %1: altitude 305 m; 2: 1205 m; 3: 2705 m; 4: 3005 m
simu.npf = 10000; %number of particles in the particle filter
simu.threshold_resample = 0.5*simu.npf;	%if the number of effective particles below threshold, do resample

%For gradient descent
% simu.LearningRate = [0.5; 0.0001];	%step for gradient movement (learning rate) (x y)
simu.alpha = 0.8;   %RMSProp
simu.iteration = 50; %number of iteration for doing gradient descent

%For feedback control
simu.kd = 0.0005; %gain for distance between UAV's position and target line
simu.kh = 1;	%gain for angle between UAV's heading and target line's slope
simu.dt = 0.1;	%sample time
simu.l = 15;	%length between two tires (bicycle model)
simu.max_vel = inf;
simu.min_vel = -inf;
simu.max_steering = inf;
simu.min_steering = -inf;

%Delay steps caused by communication
if simu.fullCommunication == 0
    simu.delayN = ceil(3*simu.N / 8);	%from the equation ()
elseif simu.fullCommunication == 1
    simu.delayN = 0;
end

%Print Simulation Parameters
simu
tic
%Initialization
run('init_generate_target_trajectory.m');
run('init_ekf.m');
run('init_pf_multiple.m');
run('init_pf_optimization.m');
run('init_save_data.m');

%Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime
    %Execute Scripts
    run('step_feedback_control.m');
    run('step_dead_reckoning.m');
    run('step_ekf.m');
    run('step_pf_multiple.m');
    % run('step_pf_withGD.m');
    run('step_save_data.m');

    %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.Ts;

    %Print Progress
    if(mod(simu.i,250)==0)
        progress=simu.accumulatedTime/(simu.simulationTime)
    end
end

%Plots
% disp('Plotting Figures...')
% run('plots_trajectories');
% run('plots_error');

%Display
% clc
clear progress
disp('Simulation Complete!')

toc
