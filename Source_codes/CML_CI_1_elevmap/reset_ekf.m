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
% disp('Initializing EKF...');
group = groups{idx_group};
num_vehicle = size(group,2);
%%

% %symbolic variables
% formulas.var = sym(zeros(1, num_vehicle*3));
% formulas.Ts = sym(sprintf('Ts'));
% for i=1:num_vehicle
%     formulas.var(i*3-2) = sym(sprintf('x%d', i));
%     formulas.var(i*3-1) = sym(sprintf('y%d', i));
%     formulas.var(i*3) = sym(sprintf('theta%d', i));
%     formulas.vel(i) = sym(sprintf('vel%d', i));
%     formulas.dtheta(i) = sym(sprintf('dtheta%d', i));
% end
% 
% %% prediction jacobian
% % disp('Generating prediction equations...');
% %prediction equations
% for i=1:num_vehicle
%     formulas.f(3*i-2,1) = formulas.var(i*3-2) + formulas.Ts*formulas.vel(i)*cos(formulas.var(i*3)+(formulas.Ts*formulas.dtheta(i)));
%     formulas.f(3*i-1,1) = formulas.var(i*3-1) + formulas.Ts*formulas.vel(i)*sin(formulas.var(i*3)+(formulas.Ts*formulas.dtheta(i)));
%     formulas.f(3*i,1) = formulas.var(i*3) + formulas.Ts*formulas.dtheta(i);
% end
% 
% formulas.fnf = matlabFunction(formulas.f);
% 
% %prediction jacobian
% % disp('Generating prediction jacobian...');
% formulas.J = jacobian(formulas.f, formulas.var);
% formulas.fnJ = matlabFunction(formulas.J);
% 
% %% observation jacobian
% % disp('Generating hypothesis equations...');
% %hypothesis equations
% formulas.M=combnk(1:num_vehicle,2); %vector of possible ranging links
% for i=1:size(formulas.M,1)
%     idx1 = formulas.M(i,1);
%     idx2 = formulas.M(i,2);
%     xidx1 = idx1*3-2;
%     xidx2 = idx2*3-2;
%     yidx1 = idx1*3-1;
%     yidx2 = idx2*3-1;
%     formulas.h(i,1) = sqrt( (formulas.var(xidx1)-formulas.var(xidx2))^2 + (formulas.var(yidx1)-formulas.var(yidx2))^2 );
% end
% formulas.fnh = matlabFunction(formulas.h);
% 
% %hypothesis jacobian
% % disp('Generating hypothesis jacobians...');
% formulas.H = jacobian(formulas.h, formulas.var);
% 
% if(simu.fullCommunication==1)
%     %Complete Communication (All Links per UAV)
%     formulas.C = []; %vector of possible combinations of measurements for updates (all possible different updates for this simulation)
%     formulas.C(1,:) = 1:size(formulas.M);
% else
%     %Pairwise Communication (1 Link per UAV)
%     if(mod(num_vehicle,2)==0) 
%         formulas.C(1,1) = find(ismember(formulas.M,[1,2],'rows'));
%         formulas.C(2,1) = find(ismember(formulas.M,[1,num_vehicle],'rows'));
%         k=2;
%         for i=3:2:num_vehicle
%             formulas.C(1,k) = find(ismember(formulas.M,[i,i+1],'rows'));
%             formulas.C(2,k) = find(ismember(formulas.M,[i-1,i],'rows'));
%             k=k+1;
%         end
%         k=1;
%         for i=1:num_vehicle/2
%             formulas.C(3,k) = find(ismember(formulas.M,[i,i+num_vehicle/2],'rows'));
%             k=k+1;
%         end
%     else
%         disp('Must have an even number of UAVs!');
%     end
% end
% 
% for i=1:size(formulas.C,1)
%     formulas.fnH{i} = matlabFunction(formulas.H(formulas.C(i,:),:));
% end

%% update management
formulas.update_counter=1;

%% update vehcile's index in hashmap
    temp = 1:size(group,2);
    formulas.map = containers.Map(temp, group);

clear i j k idx1 idx2 xidx1 xidx2 yidx1 yidx2 temp