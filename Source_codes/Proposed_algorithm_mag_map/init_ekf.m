for idx_group = 1:size(groups,2)
    group = groups{idx_group};
    num_vehicle = size(group,2);
    
    %symbolic variables
    formulas.var = sym(zeros(1, num_vehicle*3));
    formulas.Ts = sym(sprintf('Ts'));
    for i=1:num_vehicle
        formulas.var(i*3-2) = sym(sprintf('x%d', i));
        formulas.var(i*3-1) = sym(sprintf('y%d', i));
        formulas.var(i*3) = sym(sprintf('theta%d', i));
        formulas.vel(i) = sym(sprintf('vel%d', i));
        formulas.dtheta(i) = sym(sprintf('dtheta%d', i));
    end

    %% prediction jacobian
    % disp('Generating prediction equations...');
    %prediction equations
    for i=1:num_vehicle
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
    formulas.M=combnk(1:num_vehicle,2); %vector of possible ranging links
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
        if(mod(num_vehicle,2)==0) 
            formulas.C(1,1) = find(ismember(formulas.M,[1,2],'rows'));
            formulas.C(2,1) = find(ismember(formulas.M,[1,num_vehicle],'rows'));
            k=2;
            for i=3:2:num_vehicle
                formulas.C(1,k) = find(ismember(formulas.M,[i,i+1],'rows'));
                formulas.C(2,k) = find(ismember(formulas.M,[i-1,i],'rows'));
                k=k+1;
            end
            k=1;
            for i=1:num_vehicle/2
                formulas.C(3,k) = find(ismember(formulas.M,[i,i+num_vehicle/2],'rows'));
                k=k+1;
            end
        else
            disp('Must have an even number of UAVs!');
        end
    end

    for i=1:size(formulas.C,1)
        formulas.fnH{i} = matlabFunction(formulas.H(formulas.C(i,:),:));
    end
    %% ekf parameters
    ekf.P(:,:,idx_group) = repmat([1 0 0;0 1 0; 0 0 0.001],num_vehicle);   %error covariance

    % change the Q matrix based on noise
    if(simu.sigmaYawRate == 0.5*pi/180 || simu.sigmaYawRate > 0.5*pi/180)
        simu.sigmaYawRate_temp = simu.sigmaYawRate;
    else
        simu.sigmaYawRate_temp = simu.sigmaYawRate *10;
    end

    ekf.Q(:,:,idx_group) = kron(eye(num_vehicle),diag([simu.sigmaVelocity,simu.sigmaVelocity,simu.sigmaYawRate_temp/100]))*simu.Ts;  %process noise covariance
    % ekf.R = simu.updateTs*simu.sigmaRangeRatio*simu.initdistance*eye(size(formulas.C,2));
    ekf.R(:,:,idx_group) = simu.updateTs*simu.sigmaRange*eye(size(formulas.C,2));

    %% update vehcile's index in hashmap
    temp = 1:size(group,2);
    formulas.map = containers.Map(temp, group);

    %% initialize state vector
    for i=1:num_vehicle
        ekf.x(3*i-2,1,idx_group) = agent(formulas.map(i)).px(1);
        ekf.x(3*i-1,1,idx_group) = agent(formulas.map(i)).py(1);
        ekf.x(3*i,1,idx_group) = agent(formulas.map(i)).theta(1);
    end
end

clear idx_group group num_vehicle