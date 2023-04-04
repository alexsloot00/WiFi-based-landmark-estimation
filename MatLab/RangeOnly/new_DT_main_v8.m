clear all; 
clc;
close all;
% move in an ellipse/circular trajectory
circle = 1;
%%
rng(1)

isInverse = 1;
rounds = 1; % number of rounds the receiver does

% ellipse trajectory
if ~circle
    x1 = -8;
    y1 = 0;
    x2 = 8;
    y2 = 0;
    e = 0.9;
    
    a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
    b = a*sqrt(1-e^2);
    t = linspace(0,2*pi,1e4);
    X = a*cos(t);
    Y = b*sin(t);
    w = atan2(y2-y1,x2-x1);
    x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
    y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
    %true position of the landmark
    l_rad = 10; 
    l_ang = deg2rad(45);
    l_star = l_rad*[cos(l_ang); 
    sin(l_ang)];  
%     l_star = [3.90; 1.90]; 
    %Correction gain
    gamma = 1*1e1;
elseif circle
    % circular trajectory
    r = 0.1;
    t = linspace(0,2*pi,1e3);
    x = r*cos(t);
    y = r + r*sin(t);
    %true position of the landmark
    l_star = [3.90; 1.90];  
    %Correction gain
    gamma = 40;
end

p_round = zeros(2,length(x));

if isInverse
    x = flip(x);
    y = flip(y);
%     k = -k;
end

p_round(1,:) = x;
p_round(2,:) = y;

p = p_round;
% move multiple rounds
for j=1:rounds-1
    p = [p,p_round];
end

p0 = p(:,1);

% %treshold
% treshold = 0.1;

%simulation time
% sim_time = 5*1e2;

%sampling time 
T = 1e-3;

%magnitude of input 
u_mag = 1e-2;

% % initial position
% p0 = [0;
%     0]; 

%true relative position vector
z_true0 = l_star - p0;
theta_true0 = atan2(z_true0(2),z_true0(1));

%initial angle estimation theta(0)
%init_ang_err = deg2rad(-90); %in degrees
theta_est0 = deg2rad(45+180); %l_ang + init_ang_err; %-pi+2*pi*rand(1);
theta_est0 = angle_adjustment(theta_est0); 

%initial measurement
y0 = norm(z_true0);

%initial estimated landmark position
z_est0 = y0*[cos(theta_est0);
    sin(theta_est0)];
l_est0 = z_est0 + p0;

%initial unit vector of relative position z_est0 (v0)
v0 = z_est0/norm(z_est0);

%initial orthogonal vector to v0 (w0)
R_rot = [0, 1; %rotation matrix
    -1, 0];
w0 = R_rot*v0;

%initial unit vector of input direction (ui)
% ui = [-1; %cos(3*pi/4);
%     0];%sin(3*pi/4)];

%initial input
% u0 = u_mag*ui;

%inizialize vectors
%agent positions

% p = zeros(length(p0), sim_time);
%relative vectors
z_est = p; %estimated relative distance
z_true = p; %true relative distance
l_est = p; %estimated landmark position

v = p; %unit vector pointing like z_est
w = p; %unit vector hortogonal to v
v_new = p;

u = p; %input of the agent

e_l = p;    %error on landmark position
e_z = p; %error on relative position

%measures
y = zeros(1,length(p)); %true measure
y_pred = y; %predicted measure (n-1)  

e_y = y; %measures error

%angles
theta_true = zeros(1,length(p)); %true angle
theta_est = theta_true; %estimated angle

%update and landmark position
theta_update = theta_true; %updates (n-1)

theta_helper = theta_true; %variable for calculation
theta_old = theta_true; %variable for checking

e_theta = theta_true; %angle error

%condition satisfied
isSatisfied = y;
valueofSatisfied = y;
isParallel = y;
valueofParallel = y;

%initial conditions
% p(:,1) = p0;

z_est(:,1) = z_est0;
z_true(:,1) = z_true0; 
l_est(:,1) = l_est0;

v(:,1) = v0;
w(:,1) = w0;

% u(:,1) = u0;

e_l(:,1) = l_star - l_est0;
e_z(:,1) = z_true0 - z_est0;

y(1) = y0;
y_pred(1) = y0;

e_y(1) = 0;

theta_true(1) = theta_true0; 
theta_est(1) = theta_est0;

theta_update(1) = e_y(1);

theta_helper(1) = theta_est0;
theta_old(1) = theta_est0;

e_theta(1) = theta_true0 - theta_est0;

% 
% %condition initialization
% valueofSatisfied(1) = T*u(:,1)'*v(:,1) - 2*gamma*T*y(1)*u(:,1)'*w(:,1);
% valueofParallel(1) = ui'*v(:,1);
% 
% if valueofSatisfied(1) < 0 && valueofSatisfied(1) > -2
%     isSatisfied(1) = 1;
% 
% end
% 
% if valueofParallel(1) == 1 || valueofParallel(1) == -1
%     isParallel(1) = 1;
%     v_new(:,1) = [0, -1; 
%     1, 0]*v(:,1);
% end

% u(:,1) = u_mag*v_new(:,1);


%%
%simulate the evolution
for i=2:length(p)
    
    % agent is moving orthogonally to estimated landmark 
%     p(:,i) = p(:,i-1)+T*u(:,i-1);
    u(:,i-1) = p(:,i) - p(:,i-1);

    %relative position vectors evolving accordingly
    z_true(:,i) = l_star - p(:,i);
    theta_true(i) = atan2(z_true(2,i),z_true(1,i));

    z_est(:,i) = l_est(:,i-1) - p(:,i);

    %prediction of measurement 
    y_pred(i) = norm(z_est(:,i));

    %measurement
    y(i) = norm(z_true(:,i)); 

    %difference between prediction and measure
    theta_update(i) = (y(i)^2 - y_pred(i)^2); 

    theta_est(i) = theta_est(i-1) + u(:,i-1)'*w(:,i-1) + gamma*sign(u(:,i-1)'*w(:,i-1))*theta_update(i);
    theta_est(i) = angle_adjustment(theta_est(i));
    
    %relative position correction
    z_est(:,i) = y(i)*[cos(theta_est(i)); 
        sin(theta_est(i))];
    %landamrk correction
    l_est(:,i) = z_est(:,i) + p(:,i);
    
    %unit vector for next movement
    v(:,i) = z_est(:,i)/norm(z_est(:,i));
    w(:,i) = R_rot*v(:,i);

%     valueofParallel(i) = ui'*v(:,i);
% 
%     if valueofParallel(i) == 1 || valueofParallel(i) == -1
%         isParallel(i) = 1;
%         v_new(:,i) = [0, -1; 
%         1, 0]*v(:,i);
%     end

    %next input
%     u(:,i) = u0;

    %errors
    e_l(:,i) = l_star - l_est(:,i);
    e_z(:,i) = z_true(:,i) - z_est(:,i);
    
    e_y(i) = theta_update(i);

    e_theta(i) = theta_true(i) - theta_est(i);
%     e_theta(i) = angle_adjustment(e_theta(i));

%     valueofSatisfied(i) = T*u(:,i)'*v(:,i) - 2*gamma*T*y(i)*u(:,i)'*w(:,i);
% 
%     if valueofSatisfied(i) < 0 && valueofSatisfied(i) > -2
%         isSatisfied(i) = 1;
%     
%     end

        

end
%% debugging
% timelist = 1:length(theta_est);
% timelist = T * timelist;
% figure;
% plot(timelist, theta_true,'DisplayName','\theta_{true}(t)')
% hold on; 
% plot(timelist, theta_est,'DisplayName','\theta_{est}(t)')
% plot(timelist(2:end), theta_update(2:end),'DisplayName','\theta_{update}(t)');
% title('Angles over time')
% xlabel('time [s]');
% ylabel('angle [rad]');
% legend; hold off;
% 
% figure;
% plot(timelist, z_true(1,:),'DisplayName','z^x_{true}(t)')
% hold on; 
% plot(timelist, z_est(1,:),'DisplayName','z^x_{est}(t)')
% title('x component relative position vectors over time')
% xlabel('time [s]');
% ylabel('x [m]');
% legend; hold off;
% 
% figure;
% plot(timelist, z_true(2,:),'DisplayName','z^y_{true}(t)')
% hold on; 
% plot(timelist, z_est(2,:),'DisplayName','z^y_{est}(t)')
% title('y component of relative position vectors over time')
% xlabel('time [s]');
% ylabel('y [m]');
% legend; hold off;
% 
% %%
%plot of true landmark position, estimated landmark positions and agent positions over time
figure;
p(1) = plot(p(1,:),p(2,:),'r','DisplayName','p(k)'); hold on;
p(2) = plot(l_est(1,:),l_est(2,:),'bo','DisplayName','$\hat{l}(k)$'); hold on;
p(3) = plot(l_star(1,:),l_star(2,:),'gx','DisplayName','$l^*$', LineWidth=4); hold on;
title('Landmark and agent position over time')
xlabel('x [m]');
ylabel('y [m]');
axis equal;
leg1 = legend(p(1:3)); 
set(leg1,'Interpreter','latex'); 
set(leg1,'FontSize',14); hold off;
% 
% %plot of errors on estimated landmark position over time
% figure;
% e(1) = plot(timelist, e_l(1,:),'b-','DisplayName','e^x_{l}(t)'); hold on;
% e(2) = plot(timelist, e_l(2,:),'r-','DisplayName','e^y_{l}(t)'); hold on;
% title('Evolution of e_{l}(t) over time')
% xlabel('time [s]');
% ylabel('error [m]');
% legend(e(1:2)); hold off;
% 
% %plot of errors on estimated angle over time
% figure;
% plot(timelist, e_theta,'r-','DisplayName','e_{\theta}(t)'); hold on;
% title('Evolution of e_{\theta}(t) over time')
% xlabel('time [s]');
% ylabel('error [rad]');
% legend; hold off;
% 
% e_deg = rad2deg(e_theta);
% %plot of errors on estimated angle over time
% figure;
% plot(timelist, e_deg,'r-','DisplayName','$e(k)$'); hold on;
% title('Evolution of e(k) over time')
% xlabel('time [s]');
% ylabel('error [deg]');
% leg2 = legend; 
% set(leg2,'Interpreter','latex'); 
% set(leg2,'FontSize',14); hold off; 
% 
% %%
% %Kerstin plots
% xLdist = bsxfun(@minus,l_est,l_star);
% xLdist = arrayfun(@(j)norm(xLdist(:,j)),1:size(xLdist,2));
% 
% f5 = figure(5);clf(f5);set(f5,'units','centimeters', 'color', 'white','position',[0 1 20 15],'PaperPositionMode','auto');
% tiledlayout(2,1);
% nexttile;hold on;grid;
% title('\theta angle over trajectory');
% plot(theta_true);plot(theta_est);%ylim([0,pi/2]);
% nexttile;hold on;grid;
% title('distance of l^* versus l_{est} over trajectory');
% plot(xLdist,'k');
% set(gca, 'YScale', 'log');
%%
fprintf("Theta true %.2f\n", round(rad2deg(theta_true(end)),2))
fprintf("Theta estimate %.2f\n", round(rad2deg(theta_est(end)),2))
fprintf("True (x,y): %.2f, %.2f\n", l_star(1), l_star(2))
fprintf("Estimated (x,y): %.2f, %.2f\n", l_est(1,end), l_est(2,end))
