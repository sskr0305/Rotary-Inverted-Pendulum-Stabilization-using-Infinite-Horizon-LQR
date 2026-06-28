clear; clc; close all;

g = 9.81;
m = 0.127;
r = 0.216;
L = 0.337;
l = L/2;

Ip      = (1/12)*m*L^2;
I_phi   = Ip + m*l^2;
Ia      = 0.00297;
I_theta = Ia + m*r^2;
Delta   = I_theta*I_phi - (m*r*l)^2;

A = [ 0, 1, 0, 0;
      0, 0, -(m^2*g*r*l^2)/Delta, 0;
      0, 0, 0, 1;
      0, 0,  (m*g*l*I_theta)/Delta, 0 ];

B = [ 0;
      I_phi/Delta;
      0;
      -(m*r*l)/Delta ];

n = size(A,1);

Co = ctrb(A,B);
if rank(Co) < n
    warning('System is not fully controllable; LQR may fail.');
end

Q = diag([10, 1, 100, 10]);
R = 0.05;

H = [ A, -B*(1/R)*B';
     -Q,  -A'];
[V,D] = eig(H);
lambda = diag(D);
idx = find(real(lambda) < 0);
V_stable = V(:,idx);
X1 = V_stable(1:n,:);
X2 = V_stable(n+1:end,:);
P = real(X2/X1);
K = (1/R)*B'*P;

A_cl = A - B*K;

t_span = 0:0.04:10;
x0 = [0; 0; 0.15; 0];
X = zeros(length(t_span), n);

for i = 1:length(t_span)
    X(i,:) = (expm(A_cl*t_span(i))*x0).';
end

theta = X(:,1);
phi   = X(:,3);

figure('Color','w','Position',[100 100 900 400]);
subplot(1,2,1);
plot(t_span, theta, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Arm Angle');

subplot(1,2,2);
plot(t_span, phi, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('\phi (rad)');
title('Pendulum Angle (upright = 0)');

% video
v = VideoWriter('Pendulum_LQR.mp4','MPEG-4');
v.FrameRate = 25;
open(v);

fig = figure('Color','w','Position',[100 100 900 600]);
hold on; grid on; axis equal;
axis([-0.6 0.6 -0.6 0.6 -0.1 0.8]);
view(40,25);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

plot3(0,0,0,'ks','MarkerSize',12,'MarkerFaceColor','k');
hArm   = line([0 0],[0 0],[0 0],'Color','b','LineWidth',4);
hPend  = line([0 0],[0 0],[0 0],'Color','r','LineWidth',4);
hJoint = plot3(0,0,0,'ko','MarkerFaceColor','y','MarkerSize',6);
hTitle = title('');

for i = 1:length(t_span)
    th = theta(i);
    ph = phi(i);

    xj = r*cos(th);
    yj = r*sin(th);
    zj = 0;

    xt = xj - L*sin(ph)*sin(th);
    yt = yj + L*sin(ph)*cos(th);
    zt = L*cos(ph);

    set(hArm,  'XData',[0 xj], 'YData',[0 yj], 'ZData',[0 zj]);
    set(hPend, 'XData',[xj xt],'YData',[yj yt],'ZData',[zj zt]);
    set(hJoint,'XData',xj,     'YData',yj,     'ZData',zj);
    set(hTitle,'String',sprintf('Infinite-Horizon LQR | t = %.2f s', t_span(i)));

    drawnow;

    f = getframe(fig);
    if mod(size(f.cdata,1),2), f.cdata(end,:,:) = []; end
    if mod(size(f.cdata,2),2), f.cdata(:,end,:) = []; end
    writeVideo(v,f);
end

close(v);