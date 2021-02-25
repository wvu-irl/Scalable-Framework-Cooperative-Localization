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
if simu.i > simu.delayN	%do nothing in the begin due to the delay

	%Inputs For Prediction
	dthetaVec=[];
	velVec=[];
	thetaVec=[];
	xVec=[];
	yVec=[];
	for i=1:simu.N
		dthetaVec = [dthetaVec agent(i).dtheta(simu.i - simu.delayN)];
		velVec = [velVec agent(i).vel(simu.i - simu.delayN)];
		thetaVec =[thetaVec ekf.x(i*3)];
		xVec =[xVec ekf.x(i*3-2)];
		yVec =[yVec ekf.x(i*3-1)];
	end

	%Prediction
	argsJ=num2cell([simu.Ts,dthetaVec,thetaVec,velVec]);
	ekf.J = formulas.fnJ(argsJ{:});
	argsx=num2cell([simu.Ts,dthetaVec,thetaVec,velVec,xVec,yVec]);
	ekf.x = formulas.fnf(argsx{:});
	ekf.P = ekf.J*ekf.P*ekf.J' + ekf.Q; %'

	%Cleanup Prediction Vars
	clear dthetaVec velVec thetaVec xVec yVec argsJ argsx

	%Inputs For Update
	xVec=[];
	yVec=[];
	for i=1:simu.N
		xVec =[xVec ekf.x(i*3-2)];
		yVec =[yVec ekf.x(i*3-1)];
	end

	%Update
	if(mod((simu.i - simu.delayN)*simu.Ts,simu.updateTs)==0)
		argsh = num2cell([xVec,yVec]);
		h = formulas.fnh(argsh{:});
		indices=[]; %indices for state variables used in the update for calculating jacobian
		for i=1:size(formulas.C,2)
			idx = formulas.C(formulas.update_counter,i);
		    ekf.z(i,1) = agent(formulas.M(idx,1)).range(simu.i - simu.delayN,formulas.M(idx,2)); %measurements
		    ekf.h(i,1) = h(idx); %hypotheses
		    indices = [indices formulas.M(idx,:)];
		end
		indices=sort(indices);
		argsH = num2cell([xVec(unique(indices)),yVec(unique(indices))]);
		ekf.H = formulas.fnH{formulas.update_counter}(argsH{:});

		%ekf update equations
		% printP=ekf.P
		ekf.y = ekf.z - ekf.h;
		ekf.S = ekf.H*ekf.P*ekf.H' + ekf.R;	%'
		ekf.K = ekf.P*ekf.H'*inv(ekf.S);	%'
		ekf.x = ekf.x + ekf.K*ekf.y;
		ekf.P = (eye(simu.N*3)-ekf.K*ekf.H)*ekf.P;

		%chooses which update to perform (index for which ranging links to use for the update)
		formulas.update_counter = formulas.update_counter + 1;
		if(formulas.update_counter>size(formulas.C,1))
			formulas.update_counter = 1;
		end
	end

	%ekf relative
    for i=1:simu.N
   
    	%truth relative
	    % for i=1:simu.N
	    %     ekf.pi((i*3-2):(i*3)) = [agent(i).tpx(simu.i - simu.delayN)-agent(1).tpx(simu.i - simu.delayN) ...
	    %         agent(i).tpy(simu.i - simu.delayN)-agent(1).tpy(simu.i - simu.delayN) ...
	    %         agent(i).ttheta(simu.i - simu.delayN)-agent(1).ttheta(simu.i - simu.delayN)];
	    % end

	    % for i=1:simu.N
	    %     ekf.pi2((i*3-2):(i*3)) = [agent(i).tpx(simu.i - simu.delayN)-agent(2).tpx(simu.i - simu.delayN) ...
	    %         agent(i).tpy(simu.i - simu.delayN)-agent(2).tpy(simu.i - simu.delayN) ...
	    %         agent(i).ttheta(simu.i - simu.delayN)-agent(2).ttheta(simu.i - simu.delayN)];
	    % end
	    
	    %ekf relative
    	%to UAV1
        ekf.pi((i*3-2):(i*3)) = ekf.x((i*3-2):(i*3))-ekf.x(1:3);

        %to UAV2
        ekf.pi2((i*3-2):(i*3)) = ekf.x((i*3-2):(i*3))-ekf.x(4:6);

        %Cooperative Range Localization Pose (Relative Frame)
    	cl(i).rx(simu.i-simu.delayN) = ekf.pi(i*3-2);	%for UAV1
    	cl(i).ry(simu.i-simu.delayN) = ekf.pi(i*3-1);
    	%distance between each pair of UAVs is for constrains in gradian descent
    	cl(i).distance1(simu.i-simu.delayN) = sqrt(cl(i).rx(simu.i-simu.delayN)^2 + cl(i).ry(simu.i-simu.delayN)^2);

    	cl(i).rx2(simu.i-simu.delayN) = ekf.pi2(i*3-2);	%for UAV2
    	cl(i).ry2(simu.i-simu.delayN) = ekf.pi2(i*3-1);
    	cl(i).distance2(simu.i-simu.delayN) = sqrt(cl(i).rx2(simu.i-simu.delayN)^2 + cl(i).ry2(simu.i-simu.delayN)^2);
    end 
end

%Cleanup Prediction Vars
clear xVec yVec argsh argsH h idx i indices