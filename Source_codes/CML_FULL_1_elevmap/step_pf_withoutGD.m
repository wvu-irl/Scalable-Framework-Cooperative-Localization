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
        agent(i).meas = f(agent(i).tpx((simu.i - simu.delayN)), agent(i).tpy((simu.i - simu.delayN)));
        pf.meas = [pf.meas; agent(i).meas];    %[y1;y2;y3;....]
    end
    pf.meas = pf.meas + simu.sigmaMagnetic * randn(size(pf.meas));   %add magnetic measurement noise
    pf.yNow = pf.meas;  %magnetic measurement from magnetometer

    %% prediction
    % predicted movement using odom measurement
    pf.v = repmat(agent(1).vel(simu.i-simu.delayN),1,pf.npf)+simu.sigmaVelocity_temp*randn(1,pf.npf);    %noised velocity measurement
    pf.raw_v = agent(1).vel(simu.i-simu.delayN); 
    pf.dtheta = repmat(agent(1).dtheta(simu.i-simu.delayN),1,pf.npf)+simu.sigmaYawRate_temp*randn(1,pf.npf); %noised yaw rate measurement
    pf.raw_dtheta = agent(1).dtheta(simu.i-simu.delayN);
    pf.gamma = simu.sigmaGamma*randn(1,pf.npf); %randomly walk

    %[x1^1 x1^2 x1^3... ; y1^1 y1^2 ...; theta1^1 theta^2 ...; gamma^1 gamma^2 ...]
    pf.x = m_formulas.fnf(simu.Ts,pf.gamma,pf.x(3,:),pf.x(1,:),pf.x(2,:),pf.x(4,:),pf.v,pf.dtheta); 

    if(mod((simu.i-simu.delayN)*simu.Ts,simu.updateTs)==0)  %pf works when performing ranging measurement

        %obtain relative position in global frame from cooperative range localization step
        pf.position_rel = num2cell([ekf.pi(4:3:(simu.N*3-2)),ekf.pi(5:3:(simu.N*3-1))]);

        % % find local minimum of the error
        % % between measured and predicted magnetic measurement around predicted position
        % % and get good estimation of particles' positions

        % % gradient descent individual
        % for i=1:simu.N
        %     distance1_ekf(i) = cl(i).distance1(simu.i-simu.delayN);
        %     distance2_ekf(i) = cl(i).distance2(simu.i-simu.delayN);
        %     relative_heading(1,i) = ekf.pi(i*3);
        % end
        
        % [pf.x_positions] = fn_local_minimum_gradient_individual(pf.x, pf.pre_x, pf.raw_v, pf.raw_dtheta, pf.yNow, f, pf.position_rel, m_formulas, distance1_ekf, distance2_ekf,simu, relative_heading);

        % %pf.x_positions: [x1^1 x1^2 x1^3 x1^4 .... ;y1^1 y1^2 y1^3 y1^4 ... ; x2^1 x2^2 ...]
        % pf.x(1,:) = pf.x_positions(1,:);    %update x y
        % pf.x(2,:) = pf.x_positions(2,:);
        % pf.x(3,:) = atan2(pf.x(2,:)-pf.pre_x(2,:),pf.x(1,:)-pf.pre_x(1,:)); %calculate angle based on the positions

        pf.yhat = []; %estimated observation

        % this is for no gradient descent
        pf.position = m_formulas.fnh(pf.x(1,:),pf.x(2,:),pf.x(4,:),pf.position_rel{:});
        for i=1:2*pf.npf:size(pf.position,2)-2*pf.npf+1
          pf.yhat = [pf.yhat; f(pf.position(i:i+pf.npf-1)',pf.position((i+pf.npf):(i+2*pf.npf-1))')'];
        end
        
        % % this is for gradient descent individual
        % for i = 1:pf.npf
        %     pf.yhat = [pf.yhat f(pf.x_positions(1:2:end-1,i),pf.x_positions(2:2:end,i))];   %[y1^1 y1^2 y1^3 ...;y2^1 y2^3 y2^3 ...;]
        % end

        % calculate residue in pf
        pf.e = repmat(pf.yNow,1,pf.npf) - pf.yhat;

        pf.q = ones(pf.npf,1);

        for i=1:simu.N % calculate the likelihood by multiplying all UAVs' likelihood
            pf.q = pf.q .* normpdf(pf.e(i,:),0,simu.sigmaMagnetic*10)';
        end
        
        pf.q = pf.q.^(1/simu.N);
        % check divergence
%         if(sum(pf.q)<pf.divergence_threshold)
%             pf.q = zeros(pf.npf,1);
%         end

        % calculate new weights
        pf.q = pf.q .* pf.w;    %multiply previous weight
        pf.q = pf.q ./ sum(pf.q); %normalize the important weights
        pf.w = pf.q;    %copy particles' weights
        
        for i = 1:pf.nx
            pf.xf(i,1) = sum(pf.q.*pf.x(i,:)'); %average weighted 
        end

        % if the number of effective particles below threshold, re sample
        pf.N_eff = 1/sum(pf.q.^2);  %calculate effective particles

        if pf.N_eff<simu.threshold_resample
            pf.index = resampleMSV(pf.q);
            pf.x = pf.x(:,pf.index);
            pf.w = ones(pf.npf,1);
        end

        % % project back each UAVs' position using gradient descent update
        % for i = 1:simu.N
        %     pf.xf(1,i) = sum(pf.q.*pf.x_positions(2*i-1,:)');
        %     pf.xf(2,i) = sum(pf.q.*pf.x_positions(2*i,:)');
        %     pf.xf(3,i) = pf.xf(3,1) + ekf.pi(i*3);
        % end

        % project back each UAVs' position using relative position and leader's pose
        for i=2:simu.N
          pf.xf(1,i)=ekf.pi(i*3-2)*cos(pf.xf(4,1))-ekf.pi(i*3-1)*sin(pf.xf(4,1))+pf.xf(1,1);
          pf.xf(2,i)=ekf.pi(i*3-1)*cos(pf.xf(4,1))+ekf.pi(i*3-2)*sin(pf.xf(4,1))+pf.xf(2,1);
          pf.xf(3,i)=pf.xf(3,1)+ekf.pi(i*3);
        end

    else
        %without particle filter update, using dead reckoning update other UAVs' poses
        for i = 1:simu.N
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