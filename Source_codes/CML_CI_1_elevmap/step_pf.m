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
%reset pf
pf.ny = num_vehicle;	%measurement dimension (number of UAVs in the group)

if simu.i > simu.delayN %do nothing in the begin due to the delay
    %particle filter for magnetic localization with relative pose inside the group

    % save previous result
    pf.pre_xf = pf.xf;
    pf.pre_x = pf.x;

    %% particle filter
    % measurement from magnetometer
    % f is function to obtain magnetic measurement 
    % based on each vehicle's global position in magnetic anomaly map
    pf.meas = [];
    for i = 1:pf.ny
        magnetic_meas(i) = f(agent(formulas.map(i)).tpx((simu.i - simu.delayN)), agent(formulas.map(i)).tpy((simu.i - simu.delayN)));
        pf.meas = [pf.meas; magnetic_meas(i)];    %[y1;y2;y3;....]
    end
    pf.meas = pf.meas + simu.sigmaMagnetic * randn(size(pf.meas));   %add magnetic measurement noise
    pf.yNow = pf.meas;  %magnetic measurement from magnetometer

    %% prediction
    % predicted movement using odom measurement
    pf.v = repmat(agent(idx_group).vel(simu.i-simu.delayN),1,pf.npf)+simu.sigmaVelocity_temp*randn(1,pf.npf);    %noised velocity measurement
    pf.raw_v = agent(idx_group).vel(simu.i-simu.delayN); 
    pf.dtheta = repmat(agent(idx_group).dtheta(simu.i-simu.delayN),1,pf.npf)+simu.sigmaYawRate_temp*randn(1,pf.npf); %noised yaw rate measurement
    pf.raw_dtheta = agent(idx_group).dtheta(simu.i-simu.delayN);
    pf.gamma = simu.sigmaGamma*randn(1,pf.npf); %randomly walk

    %[x1^1 x1^2 x1^3... ; y1^1 y1^2 ...; theta1^1 theta^2 ...; gamma^1 gamma^2 ...]
    pf.x(:,:,idx_group) = m_formulas.fnf(simu.Ts,pf.gamma,pf.x(3,:,idx_group),pf.x(1,:,idx_group),pf.x(2,:,idx_group),pf.x(4,:,idx_group),pf.v,pf.dtheta);

    if(mod((simu.i-simu.delayN)*simu.Ts,simu.updateTs)==0)  %pf works when performing ranging measurement

        %obtain relative position in global frame from cooperative range localization step
        pf.position_rel = num2cell([ekf.relative_position(1:3:(num_vehicle*3-2)),ekf.relative_position(2:3:(num_vehicle*3-1))]);

        % % find local minimum of the error
        % % between measured and predicted magnetic measurement around predicted position
        % % and get good estimation of particles' positions

        pf.yhat = []; %estimated observation

        pf.position = m_formulas.fnh(pf.x(1,:,idx_group),pf.x(2,:,idx_group),pf.x(4,:,idx_group),pf.position_rel{:});
        for i=1:2*pf.npf:size(pf.position,2)-2*pf.npf+1
        pf.yhat = [pf.yhat; f(pf.position(i:i+pf.npf-1)',pf.position((i+pf.npf):(i+2*pf.npf-1))')'];
        end

        % calculate residue in pf
        pf.e = repmat(pf.yNow,1,pf.npf) - pf.yhat;

        pf.q(:,idx_group) = ones(pf.npf,1);

        for i=1:num_vehicle % calculate the likelihood by multiplying all UAVs' likelihood
          pf.q(:,idx_group) = pf.q(:,idx_group) .* normpdf(pf.e(i,:),0,simu.sigmaMagnetic*10)';
        end

        % check divergence
        if(sum(pf.q(:,idx_group))<pf.divergence_threshold)
          pf.q(:,idx_group) = zeros(pf.npf,1);
        end

        % calculate new weights
        pf.q(:,idx_group) = pf.q(:,idx_group) .* pf.w(:,idx_group);    %multiply previous weight
        pf.q(:,idx_group) = pf.q(:,idx_group) ./ sum(pf.q(:,idx_group)); %normalize the important weights
        pf.w(:,idx_group) = pf.q(:,idx_group);    %copy particles' weights

        for i = 1:pf.nx
          pf.xf(i,idx_group) = sum(pf.q(:,idx_group).*pf.x(i,:,idx_group)'); %average weighted
        end

        % if the number of effective particles below threshold, re sample
        pf.N_eff = 1/sum(pf.q(:,idx_group).^2);  %calculate effective particles

        if pf.N_eff<simu.threshold_resample
            
% %             test resample with CI
%         pf.x(:,:,idx_group) = repmat(pf.state_temp(:,idx_group), 1, pf.npf) ...
%        + sqrt(eye(pf.nx, pf.nx) .* pf.cov_temp(:,:,idx_group)) ...
%        *  randn(pf.nx, pf.npf);
        
          pf.index = resampleMSV(pf.q(:,idx_group));
          pf.x(:,:,idx_group) = pf.x(:,pf.index,idx_group);
          pf.w(:,idx_group) = ones(pf.npf,1);
        end
        pf.cov_temp(:,:,idx_group) = fn_particle_cov(pf.w(:,idx_group), pf.x(:,:,idx_group), pf.xf(:,idx_group));
        pf.state_temp(:,idx_group) = pf.xf(:,idx_group);
        
        pf_result(idx_group).state{1}(1,1) = pf.xf(1,idx_group);
        pf_result(idx_group).state{1}(2,1) = pf.xf(2,idx_group);
        pf_result(idx_group).state{1}(3,1) = pf.xf(3,idx_group);
%         pf_result(idx_group).heading{1}(3,1) = pf.xf(3,idx_group);
        
        pf_result(idx_group).cov{1} = pf.cov_temp(1:3,1:3,idx_group);
%         pf_result(idx_group).cov{1} = pf.cov_temp(1:2,1:2,idx_group);
        % project back each UAVs' position using relative position and leader's pose
        idx_center = find(group==idx_group); %index in subgroup.
        for i=1:num_vehicle
            if(i == idx_center) 
                continue;
            end
            pf_result(formulas.map(i)).state{end+1}(1,1) = ...
                ekf.pi(i*3-2)*cos(pf.xf(4,idx_group))-ekf.pi(i*3-1)*sin(pf.xf(4,idx_group))+pf.xf(1,idx_group);
            pf_result(formulas.map(i)).state{end}(2,1) = ...
                ekf.pi(i*3-1)*cos(pf.xf(4,idx_group))+ekf.pi(i*3-2)*sin(pf.xf(4,idx_group))+pf.xf(2,idx_group);
            pf_result(formulas.map(i)).state{end}(3,1) = ...
                pf.xf(3,idx_group)+ekf.pi(i*3);

            % test: use truth relative pose;
%             pf_result(formulas.map(i)).state{end+1}(1,1) = ...
%                 agent(formulas.map(i)).tpx(simu.i)-agent(formulas.map(idx_center)).tpx(simu.i) + pf.xf(1,idx_group);
%             pf_result(formulas.map(i)).state{end}(2,1) = ...
%                 agent(formulas.map(i)).tpy(simu.i)-agent(formulas.map(idx_center)).tpy(simu.i) + pf.xf(2,idx_group);
            
            pf_result(formulas.map(i)).cov{end+1} = pf.cov_temp(1:3,1:3,idx_group);
%             pf_result(formulas.map(i)).cov{end+1} = pf.cov_temp(1:2,1:2,idx_group);
        end

    else
        %without particle filter update, using dead reckoning update other UAVs' poses
        for i = 1:num_vehicle
            [x, y, theta] = fn_propagate_global_pose(pf.xf(1,i), ...
                                                    pf.xf(2,i), ...
                                                    pf.xf(3,i), ...
                                                    agent(i).vel(simu.i - simu.delayN), ...
                                                    agent(i).dtheta(simu.i - simu.delayN), ...
                                                    simu.Ts);

            pf.xf(1,i) = x;
            pf.xf(2,i) = y;
            pf.xf(3,i) = theta;
        end
    end
    clear i rel x y theta
end