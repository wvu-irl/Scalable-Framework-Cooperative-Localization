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
disp('Initializing EKF...');

%% initialize state vector
for i=1:simu.N
    ekf.x(3*i-2,1) = agent(i).px(1);
    ekf.x(3*i-1,1) = agent(i).py(1);
    ekf.x(3*i,1) = agent(i).theta(1);
end

%symbolic variables
formulas.var = sym(zeros(1, simu.N*3));
formulas.Ts = sym(sprintf('Ts'));
for i=1:simu.N
    formulas.var(i*3-2) = sym(sprintf('x%d', i));
    formulas.var(i*3-1) = sym(sprintf('y%d', i));
    formulas.var(i*3) = sym(sprintf('theta%d', i));
    formulas.vel(i) = sym(sprintf('vel%d', i));
    formulas.dtheta(i) = sym(sprintf('dtheta%d', i));
end

%% prediction jacobian
% disp('Generating prediction equations...');
%prediction equations
for i=1:simu.N
    formulas.f(3*i-2,1) = formulas.var(i*3-2) + formulas.Ts*formulas.vel(i)*cos(formulas.var(i*3)+(formulas.Ts*formulas.dtheta(i)));
    formulas.f(3*i-1,1) = formulas.var(i*3-1) + formulas.Ts*formulas.vel(i)*sin(formulas.var(i*3)+(formulas.Ts*formulas.dtheta(i)));
    formulas.f(3*i,1) = formulas.var(i*3) + formulas.Ts*formulas.dtheta(i);
end

formulas.fnf = matlabFunction(formulas.f);

%prediction jacobian
% disp('Generating prediction jacobian...');
formulas.J = jacobian(formulas.f, formulas.var);
formulas.fnJ = matlabFunction(formulas.J);

%% observation jacobian
% disp('Generating hypothesis equations...');
%hypothesis equations
formulas.M=combnk(1:simu.N,2); %vector of possible ranging links
for i=1:size(formulas.M,1)
    idx1 = formulas.M(i,1);
    idx2 = formulas.M(i,2);
    xidx1 = idx1*3-2;
    xidx2 = idx2*3-2;
    yidx1 = idx1*3-1;
    yidx2 = idx2*3-1;
    formulas.h(i,1) = sqrt( (formulas.var(xidx1)-formulas.var(xidx2))^2 + (formulas.var(yidx1)-formulas.var(yidx2))^2 );
end
formulas.fnh = matlabFunction(formulas.h);

%hypothesis jacobian
% disp('Generating hypothesis jacobians...');
formulas.H = jacobian(formulas.h, formulas.var);

if(simu.fullCommunication==1)
    %Complete Communication (All Links per UAV)
    formulas.C = []; %vector of possible combinations of measurements for updates (all possible different updates for this simulation)
    formulas.C(1,:) = 1:size(formulas.M);
else
    %Pairwise Communication (1 Link per UAV)
    if(mod(simu.N,2)==0) 
        formulas.C(1,1) = find(ismember(formulas.M,[1,2],'rows'));
        formulas.C(2,1) = find(ismember(formulas.M,[1,simu.N],'rows'));
        k=2;
        for i=3:2:simu.N
            formulas.C(1,k) = find(ismember(formulas.M,[i,i+1],'rows'));
            formulas.C(2,k) = find(ismember(formulas.M,[i-1,i],'rows'));
            k=k+1;
        end
        k=1;
        for i=1:simu.N/2
            formulas.C(3,k) = find(ismember(formulas.M,[i,i+simu.N/2],'rows'));
            k=k+1;
        end
    else
        formulas.C(1,1) = find(ismember(formulas.M, [1,2], 'rows'));
        formulas.C(2,1) = find(ismember(formulas.M, [1,2], 'rows'));
        formulas.C_is_valid(1,1) = 1;
        formulas.C_is_valid(2,1) = 0;
        k=2;
        for i=3:2:simu.N
            if(i==simu.N)
                formulas.C(1,k) = find(ismember(formulas.M, [i-1,i], 'rows'));
                formulas.C(2,k) = find(ismember(formulas.M, [i-1,i], 'rows'));
                formulas.C_is_valid(1,k) = 0;
                formulas.C_is_valid(2,k) = 1;
            else
                formulas.C(1,k) = find(ismember(formulas.M, [i,i+1], 'rows'));
                formulas.C(2,k) = find(ismember(formulas.M, [i-1,i], 'rows'));
                formulas.C_is_valid(1,k) = 1;
                formulas.C_is_valid(2,k) = 1;
            end
            k=k+1;
        end
        k=1;
        for i=1:simu.N/2
            formulas.C(3,k) = find(ismember(formulas.M, [i,i+(simu.N+1)/2],'rows'));
            formulas.C_is_valid(3,k) = 1;
            k=k+1;
        end
            formulas.C(3,(simu.N+1)/2) = find(ismember(formulas.M,[i,(simu.N+1)/2],'rows'));
            formulas.C_is_valid(3,(simu.N+1)/2) = 0;
    end
end

if(simu.fullCommunication~=1)
    %replace unused elements with zeros
    for i=1:size(formulas.C,1)
        for j=1:size(formulas.C,2)
            if(formulas.C_is_valid(i,j)==1)
                temC(i,j)=formulas.C(i,j);
            end
        end
    end
    
    %delete zeros
    tempRows=[];
    for i=1:size(tempC,1)
        tempR=[];
        for j=1:size(tempC,2)
            if(tempC(i,j)~=0)
                tempR=[tempR tempC(i,j)];
            end
        end
        tempRows = [tempRows; tempR];
    end
    formulas.C = tempRows;
end



for i=1:size(formulas.C,1)
    formulas.fnH{i} = matlabFunction(formulas.H(formulas.C(i,:),:));
end

%% update management
formulas.update_counter=1;

%% ekf parameters
ekf.P = repmat([1 0 0;0 1 0; 0 0 0.001],simu.N);   %error covariance

% change the Q matrix based on noise
if(simu.sigmaYawRate == 0.5*pi/180 || simu.sigmaYawRate > 0.5*pi/180)
    simu.sigmaYawRate_temp = simu.sigmaYawRate;
else
    simu.sigmaYawRate_temp = simu.sigmaYawRate *10;
end

ekf.Q = kron(eye(simu.N),diag([simu.sigmaVelocity,simu.sigmaVelocity,simu.sigmaYawRate_temp/100]))*simu.Ts;  %process noise covariance
% ekf.R = simu.updateTs*simu.sigmaRangeRatio*simu.initdistance*eye(size(formulas.C,2));
ekf.R = simu.updateTs*simu.sigmaRange*eye(size(formulas.C,2));


clear i j k idx1 idx2 xidx1 xidx2 yidx1 yidx2