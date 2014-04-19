%% Data from optimization
% [input toruqe coefficient; stroke stiffness coefficient; rotation stiffness coefficeint; deviation stiffness coefficient]

% category A
A1 = [17 61.21 21.6 50000];

A2 = [19 92.85 21.3 50000];

A3 = [21 113.68 21.5 50000];

A4 = [23 133.55 21.1 50000];

A5 = [25 149.07 21.9 50000];

A6 = [27 167.03 21.6 50000];

% category B
B1 = [17 69.74 10.56 1212.0];

B2 = [19 97.82 10.90 1231.0];

B3 = [21 120.24 11.31 1388.60];

B4 = [23 138.58 11.60 1424.4];

B5 = [25 155.98 11.66 1438.4];

B6 = [27 172.47 11.68 1482.8];


% points from each region
C1 = [21 33.92 2 70];

C2 = [21 33.92 14 210];

C3 = [21 33.92 8 1410];

C4 = [21 33.92 4 1800];




Obej = B1;


%%
Obej_tauwm = Obej(1);
Obej_kphi = Obej(2);
Obej_kpsi = Obej(3);
Obej_kth = Obej(4);

Obej_ckpi = 0;
Obej_cpsi = 0;
Obej_cth = 0;

%% global paramters
global freq c_bar R
global Ixx Iyy Izz Iyz
global c_hat r_hat mw
global kphi cphi kth cth kpsi cpsi tauwm
global rho_air
global LN g
global joint_parameters_plot
global power_plot
global result temp_rot

% loop
%close all
%clear all

%% paramters:
% mm  e-3HZ 
R = 50;                     % length (mm)
R = 3;

freq = 30e-3;               % flapping frequency (10^-3 Hz)
freq = 240e-3;

rho_wing = 1200e-6;

time = 1/freq;       % time

h = 0.045;                
rho_air = 1.23e-6;      

AR = 7;                  % wing's aspect ration
c_bar = 2*R/AR;               % mean wing chord, dimensionless
c_bar = 0.97;

St = 2*R*c_bar;
mw = rho_wing*c_bar*R*h;

mw = 2.4e-6;
h = mw/(rho_wing*c_bar*R);


%% Wing Morphology
r1 = 0.45;                  % the first-order dimensionaless radius moment
r2 = 0.929*r1^(0.732);      % Laws of shape
r3 = 0.900*r1^(0.581);

% BETA distribution parameters
p = r1 *(r1*(1-r1)/(r2^2-r1^2)-1);
q = (1-r1) *(r1*(1-r1)/(r2^2-r1^2)-1);
% number of point along wing leading edge axis
LN = 20;
dx = 1/LN;
% beta-function parameters
BETA = 0;
for i = 1 :LN
    x = i/LN;
    BETA = BETA + x^(p-1)*(1-x)^(q-1)*dx;
end
% dimensionless chord length calculated from beta distribution
for i = 1 :LN
    x = i/LN;
    c_hat(i) = x^(p-1)*(1-x)^(q-1)/BETA;
end
r_hat = 1/LN:dx:1;

if(0)
    wing_shape_generator;
end

%% Inertial of moment
z = c_hat*c_bar;
y = r_hat*R;
dy = 1/LN*R;

temp1 = 0;
for i = 1:1:LN
    temp1 = temp1 + rho_wing*h*(1/3)*z(i)^3*dy;
end
Iyy = temp1;

temp1 = 0;
for i = 1:1:LN
    temp1 = temp1 + rho_wing*h*y(i)^2*z(i)*dy;
end
Izz = temp1;

Ixx = Iyy + Izz;

temp1 = 0;
for i = 1:1:LN
    temp1 = temp1 + rho_wing*h*(1/2)*y(i)*z(i)^2*dy;
end
Iyz = temp1;



temp_rot = 0;
dr_hat = 1/LN;
for i = 1:1:LN
    temp_rot = temp_rot + r_hat(i)*c_hat(i)^2*dr_hat;
end





%% Define the number of data
num_data1 = 1;
num_data2 = 1;
num = 1;

%% Fourier Tranformation
joint_parameters = zeros(num_data1*num_data2,2);
power = zeros(num_data1*num_data2,7);
result_save = zeros(num_data1*num_data2,5);
joint_parameters_plot = zeros(1,6);
power_plot = zeros(1,7);

% check the wing trajectory stable or unstable.
Flag = zeros(num_data1*num_data2,1);

PL = zeros(num_data1*num_data2,1);
DL = zeros(num_data1*num_data2,1);
FM = zeros(num_data1*num_data2,1);

L = zeros(num_data1*num_data2,1);

% Fourier Transform
stroke = zeros(num_data1*num_data2,8);          % fourier3
deviation = zeros(num_data1*num_data2,14);      % fourier6
rotation = zeros(num_data1*num_data2,8);        % fourier3
stroke_rmse = zeros(num_data1*num_data2,1);     % check error    
deviation_rmse = zeros(num_data1*num_data2,1);
rotation_rmse = zeros(num_data1*num_data2,1);


%% hinge properties
% Stiffness and damping are dimensionless numbers.
% wing stroke stiffness and damping
kphi = Obej_kphi;
% kphi = 33;
cphi = Obej_ckpi;%0;
% wing deviation damping
% when damping is 0, start from 6.
% 0.5*(2*pi*freq)^2*Ixx is the actual value.
% 20000
% 10000

kth_initial = Obej_kth;%100*(2*pi)^2*Ixx/Ixx;%0.25*(2*pi)^2*Ixx/Ixx;
cth = Obej_cth;%0*(2*pi)*Ixx/Ixx;
% wing rotation damping
% range from 2 to 8.
% 2*(2*pi*freq)^2*Iyy is the actual value.
kpsi_initial = Obej_kpsi;%3*(2*pi)^2*Iyy/Ixx;%0.5*(2*pi)^2*Iyy/Ixx;
cpsi = Obej_cpsi;%0*(2*pi)*Iyy/Ixx;


kth = kth_initial;
kpsi = kpsi_initial;



kth_step = 10;
kpsi_step = 1;


%% Input torque
tauwm = Obej_tauwm;  % input torque.



        sprintf(' Torque     kphi     kpsi     kth/1000     cphi    cpsi    cth')
        [tauwm, kphi, kpsi, kth/1000, cphi, cpsi, cth]


%% go to the loop
% loop for kpsi
for i_2 = 1:1:num_data2
    
    % loop for kth 
    for i_1 = 1:1:num_data1
         
%         % wing stroke stiffness and damping
%         kphi = ;
%         cphi = ;
%         % wing deviation damping
%         kth = ;
%         cth = ;
%         % wing rotation damping
%         kpsi = ;
%         cpsi = ;

        % 0 0.5, 6, 3 positive 8;
        % 0 0.5, 3, 8 oval shape;
        % 0.25+7, 1.5 positive 8;



%         sprintf('      kth   kpsi  i_1  i_2, current num,  total num')
%         [kth, kpsi, i_1, i_2, num, num_data1*num_data2]
             
        
        % solve ODE
        % initial condition
        x0 = [0;0;0;0;0;0];
        % call ODE45
        % 30
        wingbeats = 39;
        [t,y] = ode45(@single_dynamics,[0 wingbeats],x0);

        last = length(t);
        x0 = y(last,:);
        wingbeats = 2;
        % 5000, 1e-12, 1e-12
        dtime = 1/1000;
        %options = odeset('RelTol',1e-6,'AbsTol',1e-6);
        [t,x] = ode45(@single_dynamics,dtime:dtime:wingbeats,x0);
        n = length(t);

        % draw the wing trajectory
        


        
        
        % calculate the aerodynamics force results
        plot_single_kinematics;
        
        % global  point1_x_LE point1_z_LE point2_x_TE point2_z_TE    

        % animation_wing_trajectory( x, n/wingbeats , R, c_bar, num, F_lw_save);   
        
        
        

        
        % Fourier Transform
        x1 = x(:,1);
        x2 = x(:,2);
        x3 = x(:,3);

        [fitresult, gof] = createFit_stroke(t, x1);
        stroke(num,:) = coeffvalues(fitresult);
        stroke_rmse(num,1) = gof.rmse;

        [fitresult, gof] = createFit_deviation(t, x2);
        deviation(num,:) = coeffvalues(fitresult);
        deviation_rmse(num,1) = gof.rmse;

        [fitresult, gof] = createFit_rotation(t, x3);
        rotation(num,:) = coeffvalues(fitresult);
        rotation_rmse(num,1) = gof.rmse;

        % rotation angle must less than 90 degreee.
        if max(x3)>0.25 || min(x3) < -0.25
            Flag(num) = 1;
        end
        
        % The frequency must be 2*pi.
        if abs(stroke(num,8)-2*pi) > 0.01
            Flag(num) = 1;
        end
        if abs(rotation(num,8)-2*pi) > 0.01
            Flag(num) = 1;
        end        
        if abs(deviation(num,14)-2*pi) > 0.01
            Flag(num) = 1;
        end

        
        % save values.
        joint_parameters(num,:) = [kth kpsi];
        power(num,:) = power_plot;
        result_save(num,:) = result;
        
        power_input = power(num,6)*(2*pi*freq^2*Ixx)*(2*pi*freq);
        PL(num) = result(2)/power_input;
        
        max_amp_stroke = 2*pi*(max(x1)-min(x1));
        wing_tip_velocity = 2*max_amp_stroke*freq*r2*R;
        reference_force = 0.5*rho_air*wing_tip_velocity^2*St;
        L(num) = result(2)/reference_force;
        
%         sprintf('      Lift   Weight  Lift Coefficient   Stable/Unstable')
%         [result(1), result(2)/g, L(num), Flag(num)]
        
        Obej_L = -PL(num)+ 1*heaviside(0.002-result(2)/g*2);
        

        
        sprintf(' Power Loading     Obej  Weight   stroke_max   stroke_min')
        [PL(num), Obej_L,  result(2)/g*2,  360*max(x1),  360*min(x1) ]
          

        result_save(num,:) = result(2)/g*2;
        
        
        num = num + 1; 
        
%         if i_1 > (num_data1-5)
%             kth_step = 0.75*(2*pi)^2*Ixx/Ixx;
%         end
        % update kth
        kth =  kth*2; 
       

    end
    % intial kth for the next loop
    kth = kth_initial;
    kth_step = 0.25*(2*pi)^2*Ixx/Ixx;
    
    if i_2 > (num_data2-0)
        kpsi_step = 0.75*(2*pi)^2*Iyy/Ixx;
    end    
    
    % update kpsi
    kpsi = kpsi + kpsi_step*2;

    
end

%% change equation forms
stroke1 = zeros(num_data1*num_data2,8);
deviation1 = zeros(num_data1*num_data2,14);
rotation1 = zeros(num_data1*num_data2,8);
% 3 -- sin
% 2 -- cos
% Please see the photo on the google drive.
for i = 1:1:(num_data1*num_data2)
    stroke1(i,1) = stroke(i,1)*360;
    stroke1(i,2) = (stroke(i,2)^2+stroke(i,3)^2)^0.5*360;
    stroke1(i,3) = -atan2(stroke(i,3),stroke(i,2))*180/pi;
    stroke1(i,4) = (stroke(i,4)^2+stroke(i,5)^2)^0.5*360;
    stroke1(i,5) = -atan2(stroke(i,5),stroke(i,4))*180/pi;
    stroke1(i,6) = (stroke(i,6)^2+stroke(i,7)^2)^0.5*360;
    stroke1(i,7) = -atan2(stroke(i,7),stroke(i,6))*180/pi;
    stroke1(i,8) = stroke(i,8);

    deviation1(i,1) = deviation(i,1)*360;
    deviation1(i,2) = (deviation(i,2)^2+deviation(i,3)^2)^0.5*360;
    deviation1(i,3) = -atan2(deviation(i,3),deviation(i,2))*180/pi;
    deviation1(i,4) = (deviation(i,4)^2+deviation(i,5)^2)^0.5*360;
    deviation1(i,5) = -atan2(deviation(i,5),deviation(i,4))*180/pi;
    deviation1(i,6) = (deviation(i,6)^2+deviation(i,7)^2)^0.5*360;
    deviation1(i,7) = -atan2(deviation(i,7),deviation(i,6))*180/pi;
    deviation1(i,8) = (deviation(i,8)^2+deviation(i,9)^2)^0.5*360;
    deviation1(i,9) = -atan2(deviation(i,9),deviation(i,8))*180/pi;
    deviation1(i,10) = (deviation(i,10)^2+deviation(i,11)^2)^0.5*360;
    deviation1(i,11) = -atan2(deviation(i,11),deviation(i,10))*180/pi;
    deviation1(i,12) = (deviation(i,12)^2+deviation(i,13)^2)^0.5*360;
    deviation1(i,13) = -atan2(deviation(i,13),deviation(i,12))*180/pi;   
    deviation1(i,14) = deviation(i,14);

    rotation1(i,1) = rotation(i,1)*360;
    rotation1(i,2) = (rotation(i,2)^2+rotation(i,3)^2)^0.5*360;
    rotation1(i,3) = -atan2(rotation(i,3),rotation(i,2))*180/pi;
    rotation1(i,4) = (rotation(i,4)^2+rotation(i,5)^2)^0.5*360;
    rotation1(i,5) = -atan2(rotation(i,5),rotation(i,4))*180/pi;
    rotation1(i,6) = (rotation(i,6)^2+rotation(i,7)^2)^0.5*360;
    rotation1(i,7) = -atan2(rotation(i,7),rotation(i,6))*180/pi;
    rotation1(i,8) = rotation(i,8);  
end

%% save data
% save data

%% 
% Deviation_Stiffness_Coefficient = joint_parameters(1,:);
% Rotation_Stiffness_Coefficient = joint_parameters(2,:);

% saveFigure;

[360*max(x1) 360*min(x1) 360*max(x3) 360*min(x3) 360*max(x2) 360*min(x2)]


%%%
[(rotation1(1,3)-stroke1(1,3))  (deviation1(1,5)-stroke1(1,3))]

sprintf('Power Coefficient')
[power(1,6)]



num_min_x1 = 0;

min_x1 = x1(1);

for j = 1:1:1000
    if (x1(j) < min_x1)
        min_x1 = x1(j);
        num_min_x1 = j;
    end

end

plot_t = zeros(1000,1);
plot_x1 = zeros(1000,1);
plot_x2 = zeros(1000,1);
plot_x3 = zeros(1000,1);
plot_Faero = zeros(1000,1);
plot_Ftran =  zeros(1000,1);
plot_Frot = zeros(1000,1);
plot_Pa = zeros(1000,1);
plot_Pi = zeros(1000,1);
plot_Pg = zeros(1000,1);
plot_Ps = zeros(1000,1);
plot_Pinput = zeros(1000,1);

plot_F_lw_save = zeros(1000,3);

for j = 1:1:1000
    plot_t(j) = t(j);
    plot_x1(j) = x1(j+num_min_x1);
    plot_x2(j) = x2(j+num_min_x1);
    plot_x3(j) = x3(j+num_min_x1);
    plot_Faero(j) = F_lw_save(j+num_min_x1,3)/(g*0.001);
    plot_Ftran(j) = F_ltran_save(j+num_min_x1,3)/(g*0.001);
    plot_Frot(j) = F_lrot_save(j+num_min_x1,3)/(g*0.001);
    
    plot_Pa(j) = Pa(j+num_min_x1);
    plot_Pi(j) = Pi(j+num_min_x1);
    plot_Pg(j) = Pg(j+num_min_x1);
    plot_Ps(j) = Ps(j+num_min_x1);
    plot_Pinput(j) = Pinput(j+num_min_x1);
    
    plot_F_lw_save(j,:) = F_lw_save(j+num_min_x1,:);
end



%%close all
%animation_wing_trajectory( [x1,x2,x3], 1000*5 , R, c_bar, num, F_lw_save);

animation_wing_trajectory( [plot_x1,plot_x2,plot_x3], 1000 , R, c_bar, num, plot_F_lw_save);

save_video_wing_trajectory( [plot_x1,plot_x2,plot_x3], 1000 , R, c_bar, num, plot_F_lw_save);


grey = [0.9,0.9,0.9];

figure(4)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-90:20:90)
axis([0 1 -90 90]);
xlabel('Stroke Cycle')
ylabel('Stroke Angle (degees)')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-90 -90],'k')

hold on
plot(plot_t,plot_x1*360,'b','LineWidth',1.5)





figure(5)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-140:30:140)
axis([0 1 -140 140]);
xlabel('Stroke Cycle')
ylabel('Rotation Angle (degees)')
hold on
%rectangle('Position',[0.5,-200,0.5,400],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-140 -140],'k')

hold on
plot(plot_t,plot_x3*360,'r','LineWidth',1.5)





figure(6)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-20:10:60)
axis([0 1 -20 60]);
xlabel('Stroke Cycle')
ylabel('Deviation Angle (degees)')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-20 -20],'k')

hold on
plot(plot_t,plot_x2*360,'g','LineWidth',1.5)



figure(7)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-3:1:8)
axis([0 1 -3 8]);
xlabel('Stroke Cycle')
ylabel('Aerodynamics Force Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-3 -3],'k')

hold on
plot(plot_t,plot_Faero,'k','LineWidth',2.5)



figure(8)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-0.6:0.3:2.5)
axis([0 1 -0.5 2.5]);
xlabel('Stroke Cycle')
ylabel('Translational Force Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-0.5 -0.5],'k')

hold on
plot(plot_t,plot_Ftran,'k--','LineWidth',2)



figure(9)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-1.5:0.3:1.5)
axis([0 1 -1.5 1.5]);
xlabel('Stroke Cycle')
ylabel('Rotational Force Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-1.5 -1.5],'k')

hold on
plot(plot_t,plot_Frot,'k--','LineWidth',2)



figure(10)
axis on
set(gca,'fontsize',20)%,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
%     'ytick',-10:5:25)
% axis([0 1 -10 25]);
xlabel('Stroke Cycle')
ylabel('Input Power Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

% hold on
% plot([0 1],[-10 -10],'k')

hold on
plot(plot_t,plot_Pinput,'k','LineWidth',2)


figure(11)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-16:2:2)
axis([0 1 -16 2]);
xlabel('Stroke Cycle')
ylabel('Aerodynamics Power Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-16 -16],'k')

hold on
plot(plot_t,plot_Pa,'k--','LineWidth',2)


figure(12)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-8:2:8)
axis([0 1 -8 8]);
xlabel('Stroke Cycle')
ylabel('Inertial Power Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-8 -8],'k')

hold on
plot(plot_t,plot_Pi,'k--','LineWidth',2)


figure(13)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1],...
    'ytick',-12:2:10)
axis([0 1 -12 10]);
xlabel('Stroke Cycle')
ylabel('Spring Power Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-12 -12],'k')

hold on
plot(plot_t,plot_Ps,'k--','LineWidth',2)


figure(14)
axis on
set(gca,'fontsize',20,'xtick',[0 0.4999 1],'xticklabel',[0 0.5 1])
axis([0 1 -3e-3 3e-3]);
xlabel('Stroke Cycle')
ylabel('Gravity Power Coefficeint')
hold on
%rectangle('Position',[0.5,-120,0.5,240],'LineStyle','none','FaceColor',grey)

hold on
plot([0 1],[-3e-3 -3e-3],'k')

hold on
plot(plot_t,plot_Pg,'k--','LineWidth',2)

