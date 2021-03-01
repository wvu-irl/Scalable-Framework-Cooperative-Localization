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
disp('Initializing PF...');
%Cooperative Magnetic Localization based magnetic map and relative positions

%chose different magnetic maps
load '../magnetic_maps/magnetic_maps.mat'
if(simu.magneticMap == 1)
    f_igrf = griddedInterpolant(xm',ym',igrf_map_305');
    f = griddedInterpolant(xm',ym',map_305');
elseif(simu.magneticMap == 2)
    f_igrf = griddedInterpolant(xm',ym',igrf_map_1205');
    f = griddedInterpolant(xm',ym',map_1205');
elseif(simu.magneticMap == 3)
    f_igrf = griddedInterpolant(xm',ym',igrf_map_2705');
    f = griddedInterpolant(xm',ym',map_2705');
elseif(simu.magneticMap == 4)
    f_igrf = griddedInterpolant(xm',ym',igrf_map_3005');
    f = griddedInterpolant(xm',ym',map_3005');
else
    disp('Wrong Input of the Magnetic Map');
end

num_vehicle = length(groups{1});    %subgroup size

pf.ny = num_vehicle;	%measurement dimension (number of UAVs in the group)

%symbolic variances
m_formulas.var_g = sym(zeros(2,num_vehicle));   %global positions
m_formulas.var_r = sym(zeros(2,num_vehicle));   %relative positions in global frame
m_formulas.Ts = sym(sprintf('Ts'));

for i=1:num_vehicle
    m_formulas.var_g(1,i) = sym(sprintf('x%d', i));
    m_formulas.var_g(2,i) = sym(sprintf('y%d', i));
    m_formulas.var_r(1,i) = sym(sprintf('xr%d', i));
    m_formulas.var_r(2,i) = sym(sprintf('yr%d', i));
end

%state vector
m_formulas.pf(1) = sym(sprintf('pfx'));
m_formulas.pf(2) = sym(sprintf('pfy'));
m_formulas.pf(3) = sym(sprintf('pftheta'));	%UAV i's heading
m_formulas.pf(4) = sym(sprintf('pgamma'));	%rotation error

m_formulas.vel = sym(sprintf('vel'));   %leader's velocity from visual inertial odom
m_formulas.vtheta = sym(sprintf('vtheta'));   %leader's yaw rate from visual inertial odom
m_formulas.gamma = sym(sprintf('gamma'));	%random walk for rotation error

%calculate each UAV's global position based on UAV i's estimated position, relative positions and rotation error angle
for i=1:2:(num_vehicle*2-1)
    m_formulas.h(i) = m_formulas.var_r(i) .* cos(m_formulas.pf(4)) - m_formulas.var_r(i+1) .* sin(m_formulas.pf(4)) + m_formulas.pf(1);
    m_formulas.h(i+1) = m_formulas.var_r(i) .* sin(m_formulas.pf(4)) + m_formulas.var_r(i+1) .* cos(m_formulas.pf(4)) + m_formulas.pf(2);
end

m_formulas.fnh = matlabFunction(m_formulas.h);

%dynamic function of UAV i
m_formulas.f(1,:) = m_formulas.pf(1) + m_formulas.Ts * m_formulas.vel .* cos(m_formulas.pf(3) + m_formulas.Ts * m_formulas.vtheta);
m_formulas.f(2,:) = m_formulas.pf(2) + m_formulas.Ts * m_formulas.vel .* sin(m_formulas.pf(3) + m_formulas.Ts * m_formulas.vtheta);
m_formulas.f(3,:) = m_formulas.pf(3) + m_formulas.Ts * m_formulas.vtheta;
m_formulas.f(4,:) = m_formulas.pf(4) + m_formulas.gamma;

m_formulas.fnf = matlabFunction(m_formulas.f);

%build model for pf
pf.nx = 4;	%state dimension (x,y,heading,gamma)
pf.npf = simu.npf;	%number of particles in the particle filter
pf.divergence_threshold = 0.001^(simu.N);
pf.P0 = zeros(pf.nx); pf.P0(1,1) = 1; pf.P0(2,2) = 1;
pf.cov_temp = zeros(pf.nx, pf.nx, simu.N);  %save cov after particle filter.
pf.state_temp = zeros(pf.nx, simu.N); %save state after particle filter.

for i = 1:simu.N
  % each uav's states
  pf.xf(:,i) = [agent(i).px(1); agent(i).py(1); agent(i).theta(1); 0];
  % each particle's states.
  pf.x(:,:,i) = repmat(pf.xf(:,i), 1, pf.npf) + pf.P0 * randn(pf.nx, pf.npf);
  % weight of particles.
  pf.w(:,i) = ones(pf.npf, 1);
  % weight of particle at each time step.
  pf.q(:,i) = ones(pf.npf, 1);
end

% change the propagate matrix based on noise
if(simu.sigmaYawRate == 0.5*pi/180 || simu.sigmaYawRate > 0.5*pi/180)
    simu.sigmaYawRate_temp = simu.sigmaYawRate;
else
    simu.sigmaYawRate_temp = simu.sigmaYawRate *10;
end

simu.sigmaVelocity_temp = simu.sigmaVelocity * 10;

clear i
