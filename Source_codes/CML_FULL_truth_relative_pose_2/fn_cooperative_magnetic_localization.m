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
%This function is used to perform parallel computing in Monte Carlo simulator
%No need to change any parameters inside
function data = cooperative_magnetic_localization(mc)

    simu.N=mc.N;    %number of UAVs
    simu.fullCommunication=mc.fullCommunication;    %1: complete communication 0: pairwise communication

    simu.simulationTime=mc.simulationTime;   %flight duration (second)
    simu.accumulatedTime=0;  %first timestep
    simu.Ts=mc.Ts;    %dead reckoning update rate (second)
    simu.updateTs=mc.updateTs;  %ranging measurements update rate and information exchange rate (second)

    simu.sigmaVelocity=mc.sigmaVelocity;  %standard deviation of velocity errors (m/s)
    simu.sigmaYawRate=mc.sigmaYawRate;    %standard deviation of yaw rate errors (rad/s)
    simu.sigmaGamma=mc.sigmaGamma;    %standard deviation of group rotation error (rad)
    %the std of ranging measurement changes based on the range, sigmaRange = sigmaRangeRatio * truth_range + delay_error
    % simu.sigmaRangeRatio=mc.sigmaRangeRatio;   %standard deviation of ranging measurement error ratio
    simu.RangeDelayError=mc.RangeDelayError; %range measurement error caused by measurement delay
    simu.sigmaRange=mc.sigmaRange;

    simu.percentNoiseDifference = mc.percentNoiseDifference; %slightly varies sigma per agent
    simu.biasVelocity=mc.biasVelocity; %standard deviation of velocity turn on bias (m/s)
    simu.biasYawRate=mc.biasYawRate; %standard deviation of yaw rate turn on bias (rad/s)
    simu.initUncertainty=mc.initUncertainty;    %standard deviation of initial position uncertainty (m) 
    simu.initoffset = mc.initoffset;    %standard deviation of initial group offset, for testing robustness of the CML (m) 
    simu.amplitude = mc.amplitude;     %amplitude of the speed (m/s) ps: base line for velocity is 50 m/s
    simu.initdistance = mc.initdistance;   %distance between each pair of neighbot UAVs' initial positions


    %For particle filter parameters
    simu.sigmaMagnetic=mc.sigmaMagnetic; %standard deviation of magnetic measurement error (nT)
    simu.magneticMap = mc.magneticMap;    %1: altitude 305 m; 2: 1205 m; 3: 2705 m; 4: 3005 m
    simu.npf = mc.npf; %number of particles in the particle filter
    simu.threshold_resample = mc.threshold_resample; %if the number of effective particles below threshold, do resample

    %For gradient descent
    simu.alpha = mc.alpha; %RMSProp
    simu.iteration = mc.iteration; %number of iteration for doing gradient descent

    %For feedback control
    simu.kd = mc.kd; %gain for distance between UAV's position and target line
    simu.kh = mc.kh;    %gain for angle between UAV's heading and target line's slope
    simu.dt = mc.dt;    %sample time
    simu.l = mc.l;    %length between two tires (bicycle model)
    simu.max_vel = mc.max_vel;
    simu.min_vel = mc.min_vel;
    simu.max_steering = mc.max_steering;
    simu.min_steering = mc.min_steering;

    %Delay steps caused by communication
    if simu.fullCommunication == 0
        simu.delayN = ceil(3*simu.N / 8);	%from the equation ()
    elseif simu.fullCommunication == 1
        simu.delayN = 0;
    end

    %Print Simulation Parameters
%     simu
    %Initialization
    run('init_generate_target_trajectory.m');
    run('init_ekf.m');
    run('init_pf.m');
    run('init_save_data.m');

    %Simulation
%     disp('Performing simulation...');
    simu.i=2;
    while simu.accumulatedTime < simu.simulationTime
        %Execute Scripts
        run('step_feedback_control.m');
        run('step_dead_reckoning.m');
        run('step_ekf.m');
        run('step_pf_withoutGD.m');
        run('step_save_data.m');

        %Update Simulation Variables
        simu.i = simu.i + 1;
        simu.accumulatedTime = simu.accumulatedTime + simu.Ts;

        %Print Progress
%         if(mod(simu.i,250)==0)
%             progress=simu.accumulatedTime/(simu.simulationTime)
%         end
    end
    
    %Save Monte Carlo Iteration
%     data.cl = cl;
%     data.mn = mn;
%     data.errors_cl = errors.cl;
%     data.errors_dr_p = errors.dr.p;
%     data.errors_mn_p = errors.mn.p;
%     data.errors_dr_theta = errors.dr.theta;
%     data.errors_mn_theta = errors.mn.theta;

    data.errors_dr = errors.dr;
    data.errors_mn = errors.mn;
    
end