close all

figure(1)
axis on
plot(x(:,1),x(:,2),'b','LineWidth',2, 'Marker','o')
hold on
set(gca,'fontsize',20,'xtick',17:2:27,'xticklabel',17:2:27)
plot(y(:,1),y(:,2),'r--','LineWidth',2, 'Marker','o')
xlabel('Input Torque Coefficient')
ylabel('Stroke Stiffness Coefficient')
axis([16.9,27.1,60,180])

figure(2)
axis on
plot(x(:,1),x(:,3),'b','LineWidth',2, 'Marker','o')
hold on
set(gca,'fontsize',20,'xtick',17:2:27,'xticklabel',17:2:27)
plot(y(:,1),y(:,3),'r--','LineWidth',2, 'Marker','o')
xlabel('Input Torque Coefficient')
ylabel('Rotation Stiffness Coefficient')
axis([16.9,27.1,10,25])

figure(3)
axis on
plot(x(:,1),x(:,4),'b','LineWidth',2, 'Marker','o')
hold on
set(gca,'fontsize',20,'xtick',17:2:27,'xticklabel',17:2:27)

plot(y(:,1),y(:,4),'r--','LineWidth',2, 'Marker','o')
xlabel('Input Torque Coefficient')
ylabel('Deviation Stiffness Coefficient')
axis([16.9,27.1,1100,1600])

figure(4)
axis on
plot(x(:,1),x(:,5),'b','LineWidth',2, 'Marker','o')
hold on
set(gca,'fontsize',20,'xtick',17:2:27,'xticklabel',17:2:27)

plot(y(:,1),y(:,5),'r--','LineWidth',2, 'Marker','o')
xlabel('Input Torque Coefficient')
ylabel('Power Loading (s/m)')
axis([16.9,27.1,0.32,0.37])

% figure(5)
% axis on
% set(gca,'fontsize',24)
% plot(x(:,1),x(:,6),'b','LineWidth',2, 'Marker','o')
% hold on
% set(gca,'fontsize',24,'xtick',17:2:27,'xticklabel',17:2:27)
% 
% plot(y(:,1),y(:,6),'r--','LineWidth',2, 'Marker','o')
% xlabel('Input Torque Coefficient')
% ylabel('deviation Stiffness Coefficient')

