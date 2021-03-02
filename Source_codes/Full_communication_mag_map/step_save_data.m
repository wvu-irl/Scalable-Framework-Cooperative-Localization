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
if simu.i > simu.delayN+1 %do nothing in the begin due to the delay
    for i=1:simu.N
        %Cooperative Magnetic Localization Pose (Global Frame)
        mn(i).px(simu.i-simu.delayN) = pf.xf(1,i);
        mn(i).py(simu.i-simu.delayN) = pf.xf(2,i);
        mn(i).theta(simu.i-simu.delayN) = pf.xf(3,i);
        % mn(i).resid(simu.i-simu.delayN) = pf.resid(i);
        
        %Dead Reckoning Error (Global Frame)
        errors.dr(i).px(simu.i-simu.delayN) = agent(i).tpx(simu.i-simu.delayN) - agent(i).px(simu.i-simu.delayN);
        errors.dr(i).py(simu.i-simu.delayN) = agent(i).tpy(simu.i-simu.delayN) - agent(i).py(simu.i-simu.delayN);
        errors.dr(i).p(simu.i-simu.delayN) = sqrt(errors.dr(i).px(simu.i-simu.delayN)^2 + errors.dr(i).py(simu.i-simu.delayN)^2);
        errors.dr(i).theta(simu.i-simu.delayN) = agent(i).ttheta(simu.i-simu.delayN) - agent(i).theta(simu.i-simu.delayN);

        %Cooperative Magnetic Localization Error (Global Frame)
        errors.mn(i).px(simu.i-simu.delayN) = agent(i).tpx(simu.i-simu.delayN) - mn(i).px(simu.i-simu.delayN);
        errors.mn(i).py(simu.i-simu.delayN) = agent(i).tpy(simu.i-simu.delayN) - mn(i).py(simu.i-simu.delayN);
        errors.mn(i).p(simu.i-simu.delayN) = sqrt(errors.mn(i).px(simu.i-simu.delayN)^2 + errors.mn(i).py(simu.i-simu.delayN)^2);
        errors.mn(i).theta(simu.i-simu.delayN) = agent(i).ttheta(simu.i-simu.delayN) - mn(i).theta(simu.i-simu.delayN);

    end
    %clean up
    clear i
end