% Cooperative Localization Simulator Main
% Author: Jared Strader and Chizhao Yang
%%
clear all; 
close all; 
clc;
rng(2)
% s = rng;    %save seed for testing later

%Simulation Parameters
simu.N=16;    %number of UAVs
simu.subN=8;  %subgroup size (4, 8, 16)
simu.fullCommunication=1;    %1: complete communication 0: pairwise communication

simu.simulationTime=60*60;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep
simu.Ts=0.1;    %dead reckoning update rate (second)
simu.updateTs=0.1;  %ranging measurements update rate and information exchange rate (second)

simu.sigmaVelocity=0.3;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.005*pi/180;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaGamma=0.001*pi/180;    %standard deviation of group rotation error (rad)
simu.sigmaRange=1;   %standard deviation of ranging measurement error (m)
%the std of ranging measurement changes based on the range, sigmaRange = sigmaRangeRatio * truth_range + delay_error
simu.sigmaRangeRatio=1/10000;   %standard deviation of ranging measurement error ratio (Data comes from GNC paper)
simu.RangeDelayError = 0; %range measurement error caused by measurement delay

simu.percentNoiseDifference = 0.01; %slightly varies sigma per agent
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
simu.initUncertainty=1;    %standard deviation of initial position uncertainty (m) 
simu.initoffset = 0;    %standard deviation of initial group offset, for testing robustness of the CML (m) 
simu.amplitude = 10;     %amplitude of the speed (m/s) ps: base line for velocity is 50 m/s
simu.initdistance = 1000;   %distance between each pair of neighbot UAVs' initial positions


%For particle filter parameters
simu.sigmaMagnetic=10; %standard deviation of magnetic measurement error (nT)
simu.magneticMap = 1;    %1: altitude 305 m; 2: 1205 m; 3: 2705 m; 4: 3005 m
simu.npf = 10000; %number of particles in the particle filter
simu.threshold_resample = 0.5*simu.npf;	%if the number of effective particles below threshold, do resample

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
    simu.delayN = ceil(3*simu.N / 8);
elseif simu.fullCommunication == 1
    simu.delayN = 0;
end

%Print Simulation Parameters
simu
% tic
%Initialization
run('init_generate_target_trajectory.m');
run('init_grouping.m');
run('init_ekf.m');
run('init_pf.m');
run('init_save_data.m');

tic
%Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime
    %Execute Scripts
    run('step_feedback_control.m');
    run('step_dead_reckoning.m');
    for idx_group = 1:size(groups,2)
        run('step_ekf.m');
        run('step_pf.m');

    end
    run('step_ci_update.m');
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
disp('Plotting Figures...')
run('plots_trajectories');
run('plots_error');

%Display
% clc
clear progress
disp('Simulation Complete!')
toc