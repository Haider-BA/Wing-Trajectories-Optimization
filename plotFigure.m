figure
plot(joint_parameters(:,1),result_save(:,1),'LineWidth',2)
xlabel('deviation stiffness coefficient')

figure
plot(joint_parameters(:,1),L(:),'LineWidth',2)
xlabel('deviation stiffness coefficient')

figure
plot(joint_parameters(:,1),PL(:),'LineWidth',2)
xlabel('deviation stiffness coefficient')

figure
plot(joint_parameters(:,1),deviation1(:,4)*360,'LineWidth',2)
xlabel('deviation stiffness coefficient')

figure
plot(joint_parameters(:,1),(deviation1(:,5)-stroke1(:,3)),'LineWidth',2)
xlabel('deviation stiffness coefficient')

saveas(figure(1),'1.png')
saveas(figure(2),'2.png')
saveas(figure(3),'3.png')
saveas(figure(4),'4.png')
saveas(figure(5),'5.png')