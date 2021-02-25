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

function [x y theta] = fn_feedback_control(x0, L, v, kd, kh, Ts, dt, l, max_vel, min_vel, max_steering, min_steering)
% this is for feedback control using bicycle model
% input:
% x0: initial pose
% L: target line
% v: target velocity
% kd: parameter for distance error (0.015)
% kh: parameter for angle error (1)
% Ts: simu.Ts
% dt: sampling time
% l: length between rare and front wheel (15)
% max_vel: max velocity limit (inf)
% min_vel: min velocity limit (-inf)
% max_steering: max steering wheel angle (inf)
% min_steering: min steering wheel angle (-inf)

%output:
% pose at current timestep

% initialization
% x0 = [0 10 0];	%initial pose (x, y, heading)
% L = [0 1 -12];	%target line ax+by+c=0
% v = 50;	%input velocity (target velocity)

%parameter
% kd=0.5;
% kh=1;

% l=5000;	%length between rare and front wheel

max_vel_limit = max_vel;
min_vel_limit = min_vel;

max_steering_wheel_angle_limit = max_steering;
min_steering_wheel_angle_limit = min_steering;

xp = x0;

%for result
% result.x = [];
% result.y = [];
% result.theta = [];
% distance = (x0(1)*L(1)+x0(2)*L(2)+L(3))/sqrt(L(1)^2+L(2)^2);
for i = 1:(Ts/dt)
	% result.x = [result.x xp(1)];
	% result.y = [result.y xp(2)];
	% result.theta = [result.theta xp(3)];

	% distance_pre = distance;
	%one controller steers the robot to minimize the robot's normal distance from the line
	distance = (xp(1)*L(1)+xp(2)*L(2)+L(3))/sqrt(L(1)^2+L(2)^2);
	% %the second controller adjusts the heading angle of the vehicle to be parallel to the line
	slope_of_line = atan2(-L(1),L(2));
	diff_ang = angdiff(slope_of_line,xp(3));
	% the second controller is related to the different of distance

	steering_wheel_angle = -kd*distance-kh*diff_ang;
	% steering_wheel_angle = -kd*distance-kh*diff_cte;

	%bicycle model
	%limit vel
	if v>max_vel_limit
		v=max_vel_limit;
	elseif v<min_vel_limit
		v=min_vel_limit;
	end

	%limit steering_wheel_angle
	if  steering_wheel_angle>max_steering_wheel_angle_limit
		steering_wheel_angle = max_steering_wheel_angle_limit;
	elseif steering_wheel_angle<min_steering_wheel_angle_limit
		steering_wheel_angle = min_steering_wheel_angle_limit;
	end
	% steering_wheel_angle
	unit_yaw_rate = tan(steering_wheel_angle) / l;

	% turn first
	yaw_rate = v*unit_yaw_rate;
	dtheta = yaw_rate*dt;

	xp(3) = xp(3) + dtheta;

	% then move
	vx = v*cos(xp(3));
	vy = v*sin(xp(3));
	% yaw_rate = v*unit_yaw_rate;

	dx = vx*dt;
	dy = vy*dt;

	xp(1) = xp(1)+dx;
	xp(2) = xp(2)+dy;

end
x = xp(1);
y = xp(2);
theta = xp(3);

% figure()
% plot(result.x,result.y); hold on;
% % plot(1:length(result.x), -L(3)*ones(1,length(result.x)),'LineWidth',5); hold on;
% grid on;

% figure()
% plot(1:length(result.theta),result.theta.*180./pi); hold on;
% grid on;

% figure()
% plot(1:length(result.x), -L(3)*ones(1,length(result.x))); hold on;
% grid on
