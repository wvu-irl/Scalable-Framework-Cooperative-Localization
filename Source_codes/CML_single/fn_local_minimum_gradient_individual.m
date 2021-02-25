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
function [x_final] = fn_local_minimum_gradient_individual(x_init, x_init_pre, raw_v, raw_dtheta, y, map, position_rel, formulas, distance1_ekf, distance2_ekf, simu, relative_heading)

	% input:
	% x_init: initial guess of all particles' states from dynamic model 4 * simu.N
	% x_init_pre: all particles' states in previous time step
	% raw_v: velocity from sensor
	% raw_dtheta
	% y: magnetic measurement of each UAV with noise simu.N * 1
	% map: the interpolation magnetic anomaly map f
	% position_rel: relative position in global frame from EKF (EKF.pi)
	% formulas: for calculating all equations, defined in init_pf.m file	
	% distance1_ekf: distances between UAV (1 2 3 ...) and UAV 1
	% distance2_ekf: distances between UAV (1 2 3 ...) and UAV 2 
	% simu
	% simu.sigmaVelocity; simu.sigmaYawRate; simu.N; threshold_distance_error=simu.sigmaRange
	% relative_heading: relative pose with respect to leader

	%output:
	%x_final: each UAV's position [x1^1 y1^1 x2^1 y2^1 x3^1 y3^1 ...; x1^2 y1^2 x2^2 y2^2 x3^2 y3^2 ...; ...]
	
	%define parameters
	delta_x = 0.5;
	delta_y = 0.1;

	x_final = zeros(2*simu.N, simu.npf);

	%for limitation in gradient descent movement
	max_theta_noise = 3*simu.sigmaYawRate*simu.Ts;
	max_velocity_nosie = 3*simu.sigmaVelocity*simu.Ts;
	raw_movement = raw_v*simu.Ts;

	for i = 1:simu.npf	%do it for each particle

		x_process = x_init(:,i);	%initial guess of leader
		x_pre_process = x_init_pre(:,i);
		x_process(3) = x_pre_process(3)+raw_dtheta*simu.Ts;
		x_init_heading = repmat(x_process(3),1,simu.N) + relative_heading;

		% propagate to other UAVs' position using relative position from EKF
		position = formulas.fnh(x_process(1), x_process(2), x_process(4), position_rel{:}); %[x1 y1 x2 y2 x3 y3 ...]
		position = position';

		% G function (error of magnetic measurement)
		yhat = map(position(1:2:end-1), position(2:2:end));
		G = yhat - y;	% general function [f(x1,y1) - y1; f(x2,y2) -y2; ...]

		%object function
		F = 0.5.*G.^2;	%simu.N by 1

		%lock (if movement over threshold, save previous result, and lock the value)
		lock=zeros(1,simu.N);

		% for k = 1:simu.N
		% 	x(2*k-1) = position(2*k-1);
		% 	x(2*k) = position(2*k);
		% end

		x = position;
		sigma = zeros(1, 2*simu.N)';	%RMSProp
		for j=1:simu.iteration	%for limited times iteration
			F_pre = F;
			position_pre = position;
			x_pre = x;

			%numerical Jacobian matrix calculation
			position_positive_x = position + repmat([delta_x;0],simu.N,1);
			yhat_positive_x = map(position_positive_x(1:2:end-1), position_positive_x(2:2:end));

			% position_negative_x = position - repmat([delta_x;0],simu.N,1);
			% yhat_negative_x = map(position_negative_x(1:2:end-1), position_negative_x(2:2:end));

			position_positive_y = position + repmat([0;delta_y],simu.N,1);
			yhat_positive_y = map(position_positive_y(1:2:end-1), position_positive_y(2:2:end));

			% position_negative_y = position - repmat([0;delta_y],simu.N,1);
			% yhat_negative_y = map(position_negative_y(1:2:end-1), position_negative_y(2:2:end));

			%[dx1 dy1;dx2 dy2;dx3 dy3;...]
			J = [((yhat_positive_x - yhat)./(delta_x)) ((yhat_positive_y - yhat)./(delta_y))];

			delta_F = [J(:,1).*G J(:,2).*G];
			delta_F = delta_F';

			% inv_delta_F = ones(8,1);
			inv_delta_F = [];
			for k=1:simu.N
				inv_delta_F = [inv_delta_F; delta_F(:,k)];

			end 	%[dx1;dy1;dx2;dy2;dx3;dy3;....]
			move_step = simu.LearningRate.*inv_delta_F;
			oversize_index = find(abs(move_step) > abs(simu.LearningRate));
			move_step(oversize_index) = simu.LearningRate(oversize_index).*move_step(oversize_index)./abs(move_step(oversize_index));
			% if j==1
			% 	sigma = inv_delta_F;	%RMSProp
			% else
			% 	sigma = sqrt(simu.alpha.*(sigma.^2) + (1-simu.alpha).*(inv_delta_F.^2));
			% end

			% calculate each UAV's position in next step
			% position = position - simu.LearningRate./sigma.*inv_delta_F;
			% position = position - simu.LearningRate.*inv_delta_F;
			% move_step
			position = position - move_step;
			% update G function
			yhat = map(position(1:2:end-1), position(2:2:end));
			G = yhat-y;

			% object function
			F = 0.5.*G.^2; %simu.N by 1

			% check the contraction from dynamic function for leader
			if(lock(1) == 0)

				r = (position(1)-x_pre_process(1))^2 + (position(2)-x_pre_process(2))^2;

				if(F(1) > F_pre(1) || ...
					~( ...
					(((position(1)-x_pre_process(1))*cos(x_init_heading(1))+ (position(2)-x_pre_process(2))*sin(x_init_heading(1))) > sqrt(r)*cos(max_theta_noise)) & ...
					le(r,(raw_movement + max_velocity_nosie)^2) & ...
					ge(r,(raw_movement - max_velocity_nosie)^2) ...
					))
					x(1) = position_pre(1);
					x(2) = position_pre(2);
					lock(1) = 1;
					
				else
					x(1) = position(1);
					x(2) = position(2);
				end
			end

			for k=2:simu.N
				if(lock(k) == 0)
					if(F(k) > F_pre(k))
						x(2*k-1) = position_pre(2*k-1);
						x(2*k) = position_pre(2*k);
						lock(k) = 1;
					else
						x(2*k-1) = position(2*k-1);
						x(2*k) = position(2*k);
					end
				end
				pi1((k*2-1):(k*2)) = x((2*k-1):(2*k))-x(1:2);
				pi2((k*2-1):(k*2)) = x((2*k-1):(2*k))-x(3:4);

				rx1(k) = pi1(2*k-1);
				ry1(k) = pi1(2*k);

				distance1(k) = sqrt(rx1(k)^2+ry1(k)^2);

				rx2(k) = pi2(2*k-1);
				ry2(k) = pi2(2*k);

				distance2(k) = sqrt(rx2(k)^2+ry2(k)^2);

				error1(k) = abs(distance1(k) - distance1_ekf(k));
				error2(k) = abs(distance2(k) - distance2_ekf(k));
			end

			pi1((1):(2)) = x((1):(2))-x(1:2);
			pi2((1):(2)) = x((1):(2))-x(3:4);

			rx1(1) = pi1(1);
			ry1(1) = pi1(2);

			distance1(1) = sqrt(rx1(1)^2+ry1(1)^2);

			rx2(1) = pi2(1);
			ry2(1) = pi2(2);

			distance2(1) = sqrt(rx2(1)^2+ry2(1)^2);

			error1(1) = abs(distance1(1) - distance1_ekf(1));
			error2(1) = abs(distance2(1) - distance2_ekf(1));


			% %check distances
			% for k=1:simu.N
			% 	pi1((k*2-1):(k*2)) = x((2*k-1):(2*k))-x(1:2);
			% 	pi2((k*2-1):(k*2)) = x((2*k-1):(2*k))-x(3:4);

			% 	rx1(k) = pi1(2*k-1);
			% 	ry1(k) = pi1(2*k);

			% 	distance1(k) = sqrt(rx1(k)^2+ry1(k)^2);

			% 	rx2(k) = pi2(2*k-1);
			% 	ry2(k) = pi2(2*k);

			% 	distance2(k) = sqrt(rx2(k)^2+ry2(k)^2);

			% 	error1(k) = abs(distance1(k) - distance1_ekf(k));
			% 	error2(k) = abs(distance2(k) - distance2_ekf(k));
			% end

			if((sum(error1>simu.sigmaRange) + sum(error2>simu.sigmaRange)) >0)
				x = x_pre;
				break;
				
			end

			if(all(lock) == 1)
				break;
			end
			
		end
		
		x_final(:,i) = x;
	end



