% clustring vehicle using distances.
% params:
% agents' truth position
% time step i
% epsilon distance (min distance)
clear groups;
vertex = zeros(simu.N, 2);
for i = 1:simu.N
    vertex(i,1) = agent(i).tpx(simu.i); 
    vertex(i,2) = agent(i).tpy(simu.i);
end

D = pdist2(vertex, vertex); % get pairwise distance.

for i = 1:simu.N
    neighbors = find(D(i,:)<=simu.epsilon);
%     groups{i} = neighbors;
    groups{i} = [1 2 3 4];
end

clear i D vertex neighbors;

