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
% Cooperative Localization Monte Carlo Simulator 
% Author: Jared Strader and Chizhao Yang
clear; close all; clc;

%Monte Carlo Parameters
%all parameters are introduced in detail in main.m 
mc.N=32;    %number of UAVs
mc.fullCommunication=1;    %1: complete communication 0: pairwise communication

mc.simulationTime=1*60;   %flight duration (second)
mc.accumulatedTime=0;  %first timestep
mc.Ts=0.1;    %visual inertial odometry update rate (second)
mc.updateTs=0.1;  %ranging measurements update rate and information exchange rate (second)

mc.sigmaVelocity=0.3;  %standard deviation of velocity errors (m/s)
mc.sigmaYawRate=0.005*pi/180;    %standard deviation of yaw rate errors (rad/s)
mc.sigmaGamma=0.001*pi/180;    %standard deviation of group rotation error (rad)
%the std of ranging measurement changes based on the range, sigmaRange = sigmaRangeRatio * truth_range + delay_error
% mc.sigmaRangeRatio=1/10000;   %standard deviation of ranging measurement error ratio (Data comes from GNC paper)
mc.RangeDelayError = 0; %range measurement error caused by measurement delay
mc.sigmaRange = 1;

mc.percentNoiseDifference = 0.01; %slightly varies sigma per agent
mc.biasVelocity=0.1*mc.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
mc.biasYawRate=0.1*mc.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
mc.initUncertainty=1;    %standard deviation of initial position uncertainty (m) 
mc.initoffset = 0;    %standard deviation of initial group offset, for testing robustness of the CML (m) 
mc.amplitude = 10;     %amplitude of the speed (m/s) 
mc.initdistance = 1000;   %distance between each pair of neighbot UAVs' initial positions

%For particle filter parameters
mc.sigmaMagnetic=10; %standard deviation of magnetic measurement error (nT)
mc.magneticMap = 1;    %1: altitude 305 m; 2: 1205 m; 3: 2705 m; 4: 3005 m
mc.npf = 10000; %number of particles in the particle filter
mc.threshold_resample = 0.5*mc.npf;	%if the number of effective particles below threshold, do resample

%For gradient descent
mc.alpha = 0.8;	%RMSProp
mc.iteration = 50; %number of iteration for doing gradient descent

%For feedback control
mc.kd = 0.0005; %gain for distance between UAV's position and target line
mc.kh = 1;    %gain for angle between UAV's heading and target line's slope
mc.dt = 0.1;    %sample time
mc.l = 15;    %length between two tires (bicycle model)
mc.max_vel = inf;
mc.min_vel = -inf;
mc.max_steering = inf;
mc.min_steering = -inf;

mc.totalInterations = 4;  %number of interation

%edit parameters below if you want to perform other sensitive analysis
 mc.N_pool = [4 8];
% mc.sigmaVelocity_pool = [0.003 0.03 0.3 3];
% mc.sigmaYawRate_pool = [0.0005*pi/180 0.005*pi/180 0.05*pi/180 0.5*pi/180];
% mc.initoffset_pool = [10 100];
% mc.sigmaMagnetic_pool = [1 10 50 100];

% mc.TotalIndex = length(mc.sigmaVelocity_pool);
% mc.TotalIndex = length(mc.sigmaYawRate_pool);
% mc.TotalIndex = length(mc.sigmaMagnetic_pool);
mc.TotalIndex = length(mc.N_pool);

for TotalIndex = 1:mc.TotalIndex    
    mc.N = mc.N_pool(TotalIndex); 
    % mc.sigmaVelocity = mc.sigmaVelocity_pool(TotalIndex);
     % mc.sigmaMagnetic = mc.sigmaMagnetic_pool(TotalIndex);
%     mc.sigmaYawRate = mc.sigmaYawRate_pool(TotalIndex); 
    mc.biasVelocity=0.1*mc.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
	mc.biasYawRate=0.1*mc.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
    clear data;
    parfor iteration = 1: mc.totalInterations
    	s = RandStream('mt19937ar', 'seed', sum(100*(1+iteration/mc.totalInterations)*clock));	%fix seed bug
		RandStream.setGlobalStream(s)
        iteration 	%show iteration number
        data(iteration) = fn_cooperative_magnetic_localization(mc);       
    end
 	%edit mat file name when saving different data   
    % save(['mc_data_sensitive_Velocity',num2str(mc.sigmaVelocity),'_withoutGD_Map1'], 'data','-v7.3');
     % save(['mc_data_sensitive_sigmaMagnetic',num2str(mc.sigmaMagnetic),'_withoutGD_Map1'], 'data','-v7.3');
%     save(['mc_data_sensitive_sigmaYawRate',num2str(mc.sigmaYawRate*180/pi),'_withoutGD_Map1'], 'data','-v7.3');
    save(['mc_data_sensitive_num_of_UAV',num2str(mc.N),'_Full_connection_real_relative'], 'data','-v7.3');
end
