import gtsam.*
graph = NonlinearFactorGraph;

% use uav 1 as prior factor
priorMean = Pose2(pf.xf(1,1), pf.xf(2,1), pf.xf(3,1));
priorNoise = noiseModel.Diagonal.Sigmas([1;  1; 0.01]);
graph.add(PriorFactorPose2(1, priorMean, priorNoise));

rangingNoise = noiseModel.Isotropic.Sigma(1, simu.sigmaRange);

% add ranging measurements
for uav_index1 = 1:simu.N
  for uav_index2 = uav_index1:simu.N
    graph.add(RangeFactorPose2(uav_index1, uav_index2, agent(uav_index1).range(simu.i - simu.delayN, uav_index2), rangingNoise));
  end
end

% add other uavs' pose estimates
initialEstimate = Values;
for uav_num = 1:simu.N
  initialEstimate.insert(uav_num, Pose2(pf.xf(1,uav_num), pf.xf(2,uav_num), pf.xf(3,uav_num)));
end
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
% result.print(sprintf('\nFinal result:\n  '));

poses_optimized = utilities.extractPose2(result);


% temp code.
for uav_num = 1:simu.N
  pf.xf(1,uav_num) = poses_optimized(uav_num,1);
  pf.xf(2,uav_num) = poses_optimized(uav_num,2);
  pf.xf(2,uav_num) = poses_optimized(uav_num,2);
end
