clear
close all

import gtsam.*

graph = NonlinearFactorGraph;

priorMean = Pose2(0.0, 0.0, 0.0);
priorNoise = noiseModel.Diagonal.Sigmas([0.1;  0.1; 0.1]);
graph.add(PriorFactorPose2(1, priorMean, priorNoise));

rangingNoise = noiseModel.Isotropic.Sigma(1,0.01);

% graph.add(RangeFactor<Pose2, Point2>(1, 2, 2.0, rangingNoise));
% graph.add(RangeFactor<Pose2, Point2>(1, 3, 4.5, rangingNoise));

graph.add(RangeFactorPose2(1, 2, 2.11, rangingNoise));
graph.add(RangeFactorPose2(1, 3, 4.51, rangingNoise));
graph.add(RangeFactorPose2(1, 4, 7.07, rangingNoise));
% 
% graph.add(RangeFactorPose2(2, 3, 2.51, rangingNoise));
% graph.add(RangeFactorPose2(2, 4, 5.00, rangingNoise));
% graph.add(RangeFactorPose2(3, 4, 2.57, rangingNoise));

initialEstimate = Values;
initialEstimate.insert(1, Pose2(0.1, 0.0, 0.1));
initialEstimate.insert(2, Pose2(0.8, 2.2, -0.2));
initialEstimate.insert(3, Pose2(0.3, 4.7, 0.2));
initialEstimate.insert(4, Pose2(1.2, 6.8, -0.1));

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));
% result.print();

%%
cla;
hold on;

plot2DTrajectory(result); hold on;
plot(0.1, 0.0,'r*'); hold on;
plot(0.8, 2.2,'r*'); hold on;
plot(0.3, 4.7,'r*'); hold on;
plot(1.2, 6.8,'r*'); hold on;
a=plot([NaN],[NaN], 'r*'); hold on;
plot(0.0, 0.0,'bo'); hold on;
plot(0.7, 2.0,'bo'); hold on;
plot(0.4, 4.5,'bo'); hold on;
plot(1.0, 7.0,'bo'); hold on;
b=plot([NaN],[NaN], 'bo'); hold on;
c=plot([NaN],[NaN], 'k*'); hold on;

legend([a; b; c],{'Estimate', 'Truth', 'Optimized'});
grid on

axis equal
% view(2)