function Obej_L= fun(Obej)

Obej_tauwm = Obej(1)*10;
Obej_kphi = Obej(2)*100;
Obej_kpsi = Obej(3)*10;
Obej_kth = 50000;

Obej_ckpi = 0;
Obej_cpsi = 0;
Obej_cth = 200;

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

h = 0.05;                
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



%         sprintf(' Torque     kphi     kpsi     kth/1000')
%         [tauwm, kphi, kpsi, kth/1000]


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
        wingbeats = 4;
        [t,y] = ode45(@single_dynamics,[0 wingbeats],x0);

        last = length(t);
        x0 = y(last,:);
        wingbeats = 3;
        % 5000, 1e-12, 1e-12
        dtime = 1/1000;
        %options = odeset('RelTol',1e-6,'AbsTol',1e-6);
        [t,x] = ode45(@single_dynamics,dtime:dtime:wingbeats,x0);
        n = length(t);

        % draw the wing trajectory
        % animation_wing_trajectory( x, n , R, c_bar, num);

        % calculate the aerodynamics force results
        plot_single_kinematics;
        
        
        % Fourier Transform
        x1 = x(:,1);
        x2 = x(:,2);
        x3 = x(:,3);

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
        
        Obej_L = -PL(num);
        

        
%         sprintf(' Power Loading     Obej  Weight   stroke_max   stroke_min')
%         [PL(num), Obej_L,  result(2)/g*2,  360*max(x1),  360*min(x1) ]
          

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


%% save data
% save data

%% 
% Deviation_Stiffness_Coefficient = joint_parameters(1,:);
% Rotation_Stiffness_Coefficient = joint_parameters(2,:);








% saveFigure;
end