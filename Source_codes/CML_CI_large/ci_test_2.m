% x(:,1) = [1;2;0];
% x(:,2) = [2;2;0];
% x(:,3) = [2;3;0];
% 
% p(:,:,1) = [10 5 0; 5 10 0; 0 0 1];
% p(:,:,2) = [10 -5 0; -5 10 0; 0 0 1]; 
% p(:,:,3) = [12 9 0; 9 12 0; 0 0 1]; 
% 
% [fusedState,fusedCov] = fusecovint(x,p);
% 
% tPlotter = theaterPlot('XLim',[-10 10],'YLim',[-10 10],'ZLim',[-10 10]);
% tPlotter1 = trackPlotter(tPlotter, ...
%     'DisplayName','Input Tracks','MarkerEdgeColor',[0.000 0.447 0.741]);
% tPlotter2 = trackPlotter(tPlotter,'DisplayName', ...
%     'Fused Track','MarkerEdgeColor',[0.850 0.325 0.098]);
% plotTrack(tPlotter1,x',p)
% plotTrack(tPlotter2,fusedState',fusedCov)
% title('Covariance Intersection Fusion')

%{
Franken, Dietrich, and Andreas Hupper. 
"Improved fast covariance intersection for distributed data fusion." 
2005 7th International Conference on Information Fusion. 
Vol. 1. IEEE, 2005.
%}


clear all

a = [1;2;0.1];
b = [1;2;0.3];
d = [1;1;0.2];

% a = [1;2;0];
% b = [20;2;0];

% w=0.5;
n=3;

A = [10 5 0; 5 10 0; 0 0 2];
B = [10 -5 0; -5 10 0; 0 0 4];
D = [5 -1 0; -5 10 0; 0 0 3];

Ai = inv(A);
Bi = inv(B);
Di = inv(D);

Information{1} = Ai;
Information{2} = Bi;
Information{3} = Di;

sumi = Ai+Bi+Di;

sum_over_i = det(Ai) - det(sumi-Ai) + det(Bi) - det(sumi-Bi) + det(Di) - det(sumi-Di);

% for i=1:n
%     w(i,1) = (det(sumi) - det(sumi- Information{i}) + det(Information{i})) ...
%         / (n*det(sumi) + sum_over_i);
% end


tr(1) = trace(A);
tr(2) = trace(B);
tr(3) = trace(D);

tr = 1./tr;

sum_tr = sum(tr);

for i=1:n
    w(i,1) = tr(i)/sum_tr;
end

Ci = w(1)*Ai + w(2)*Bi + w(3)*Di;
C = inv(Ci);
c = C*(w(1)*Ai*a + w(2)*Bi*b + w(3)*Di*d);
% Ci = w*Ai + (1-w)*Bi;
% C = inv(Ci);
% c = C*(w*Ai*a+(1-w)*Bi*b);

% Ci = w*(Ai + Bi +Di);
% C = inv(Ci);
% c = C*w*(Ai*a+Bi*b+Di*d);

% PA = A + (c-a)*(c-a)';
% PB = B + (c-b)*(c-b)';
% PD = D + (c-d)*(c-d)';

% for i=1:3
%     for j=1:3
%         array = [PA(i,j),PB(i,j),PD(i,j)];
%         [M,I] = max([abs(PA(i,j)),abs(PB(i,j)),abs(PD(i,j))]);
%         C(i,j) = array(I);
%     end
% end



figure()
ha = plot_gaussian_ellipsoid(a,A); hold on;
hb = plot_gaussian_ellipsoid(b,B); hold on;
hd = plot_gaussian_ellipsoid(d,D); hold on;
hc = plot_gaussian_ellipsoid(c,C); hold on;

% hc=error_ellipse(C(1:2,1:2),c(1:2),'style','--r'); hold on;
% hb=error_ellipse(B(1:2,1:2),b(1:2),'style','b'); hold on;
% ha=error_ellipse(A(1:2,1:2),a(1:2),'style','k'); hold on;
% hd=error_ellipse(D(1:2,1:2),d(1:2),'style','g'); hold on;
% plot(c(1),c(2),'*r')
% plot(b(1),b(2),'.b')
% plot(a(1),a(2),'.k')
% plot(d(1),d(2),'.g')
% grid on;
% legend('Combined', 'B', 'A','D');
% title('Covariance Fusion using Covariance Intersection (A,B,D are input)');

% figure()
% error_ellipse(C,c,'style','--r'); hold on;
% error_ellipse(B,b,'style','b'); hold on;
% error_ellipse(A,a,'style','k'); hold on;
