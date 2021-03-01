% plot magnetic anomaly map with altitude at 305 meters
% author: Chizhao Yang

clear
close all

load magnetic_maps.mat
fig1=figure();
mesh(xm_lon,ym_lat,map_305); hold on;
c=colorbar('FontSize',10);
c.Label.String ='Magnetic Anomaly Measurement (nT)';
axis([-91.2 -78.3 38.24 41.78 -inf inf]);
xlabel('Longitude (degree)');
ylabel('Latitude (degree)');
set(gca,'fontsize',10);
title('Magnetic Anomaly Map')
view(0,90)