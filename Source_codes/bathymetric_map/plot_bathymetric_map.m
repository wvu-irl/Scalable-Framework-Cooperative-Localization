% plot bathymetric map
% author: Chizhao Yang

clear all
close all

[A,R] = readgeoraster('chancen_bath.asc');

% remove nan rows and cols
I = A ~= -9999;
A = I .* A;
A(~any(A,2), :) = [];
A(:, ~any(A,1)) = [];

% change rows in map data
a = A;
for i = 1:size(a,1)/2
    a([i size(a,1)-i], :) = a([size(a,1)-i i], :);
end

% assign alt lon to data
ll =  -119.643628 : (-119.471725 + 119.643628) / (size(A,2) - 1) : -119.471725;
aa = 34.320467 : (34.390598 - 34.320467) / (size(A,1) - 1) : 34.390598;
[lm, am] = meshgrid(ll, aa);

f = griddedInterpolant(lm',am', a');

figure()
mesh(lm,am,a)
hold on
axis([-119.643 -119.473 34.3207 34.39 -70 -30])
h =colorbar('FontSize',12);
h.Label.String ='Depth (meter)';
xlabel('Longitude (deg)')
ylabel('Latitude (deg)')
zlabel('Depth (meter)')
title('Bathymetric Map')
view(0,90)
set(gca,'fontsize',12);
set(gcf,'position',[100 100 20000 300]);
% print('Bathymetric_map','-dpng','-r300')


