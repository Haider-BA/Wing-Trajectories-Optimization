%% Plotting simulation results

%% Global Parameters
global freq c_bar R
global Ixx Iyy Izz Iyz
global c_hat r_hat mw
global kphi cphi kth cth kpsi cpsi tauwm

global rho_air

global LN g

global joint_parameters_plot
global power_plot
global result temp_rot

%% Initial values.
n = length(t);

x_dot = zeros(n,6);

tau = zeros(n,3);


AoA_save = zeros(n,1);

Mzs = zeros(n,1);
Mzd = zeros(n,1);
Mz = zeros(n,3);
Mzs_save = zeros(n,3);
Mzd_save = zeros(n,3);

Mxs = zeros(n,1);
Mxd = zeros(n,1);
Mx = zeros(n,3);
Mxs_save = zeros(n,3);
Mxd_save = zeros(n,3);

Mys = zeros(n,1);
Myd = zeros(n,1);
My = zeros(n,3);
Mys_save = zeros(n,3);
Myd_save = zeros(n,3);


F_tran_save = zeros(n,3);
F_rot_save = zeros(n,3);

F_aero_save = zeros(n,3);

F_drag_save = zeros(n,1);

F_lw = zeros(n,3);
F_lw_save = zeros(n,3);

F_ltran_save = zeros(n,3);
F_lrot_save = zeros(n,3);

M_tran_save = zeros(n,3);
M_rot_save = zeros(n,3);

Maero_save = zeros(n,3);
Maero_save_0 = zeros(n,3);
Maero_save_1 = zeros(n,3);
Maero_save_2 = zeros(n,3);

ratio = zeros(n,1);




M_g_save = zeros(n,3);


Pa = zeros(n,1);
Pi = zeros(n,1);
Pg = zeros(n,1);
Ps = zeros(n,1);
Pd = zeros(n,1);
Pinput = zeros(n,1);
Psum = zeros(n,1);

Pxd = zeros(n,1);
Pyd = zeros(n,1);





%% wing moment of inertia matrix
I_wing = [  1   0      0;
            0   Iyy/Ixx    Iyz/Ixx;
            0   Iyz/Ixx    Izz/Ixx];

%% Loop
for i = 1:1:n;
    
    
%% sine,cosine function
s2 = sin(2*pi*x(i,2));
c2 = cos(2*pi*x(i,2));
s3 = sin(2*pi*x(i,3));
c3 = cos(2*pi*x(i,3));
%% Constraint Equation
    % left wing angular velocity
    W10 = [c2*c3 s3 0;
          -c2*s3 c3 0;
           s2 0 1];
    temp = (W10)\[x(i,4);x(i,5);x(i,6)];
    
    x_dot(i,1) = temp(1);
    x_dot(i,2) = temp(2);
    x_dot(i,3) = temp(3);

%% Hinge Torques
    % P1: left wing and main body hinge
    % Active hinge torques, along z direction
    % input signal
    tau(i,:) = transpose(Ry(2*pi*x(i,3)))*transpose(Rx(2*pi*x(i,2)))*[0;0;tauwm*sin(2*pi*t(i))];
    
    % stroke
    Mzs(i,:) = -kphi*x(i,1);
    Mzd(i,:) = -cphi*x_dot(i,1);
    
    Mzs_save(i,:) = transpose(Ry(2*pi*x(i,3)))*transpose(Rx(2*pi*x(i,2)))*[0;0;Mzs(i,:)];
    Mzd_save(i,:) = transpose(Ry(2*pi*x(i,3)))*transpose(Rx(2*pi*x(i,2)))*[0;0;Mzd(i,:)];
    
    Mz(i,:) = Mzs_save(i,:)+Mzd_save(i,:);

    % Passive hinge torques, along x direction  (negative direction)
    % deviation
    Mxs(i,:) = -kth*x(i,2);
    Mxd(i,:) = -cth*x_dot(i,2);
    
    Mxs_save(i,:) = transpose(Ry(2*pi*x(i,3)))*[Mxs(i,:);0;0];
    Mxd_save(i,:) = transpose(Ry(2*pi*x(i,3)))*[Mxd(i,:);0;0];
    
    Mx(i,:) = Mxs_save(i,:)+Mxd_save(i,:);

    % Passive hinge torques, along y direction
    % rotation
    Mys(i,:) = -kpsi*x(i,3);
    Myd(i,:) = -cpsi*x_dot(i,3);
    
    Mys_save(i,:) = [0;Mys(i,:);0];
    Myd_save(i,:) = [0;Myd(i,:);0];
    
    My(i,:) = Mys_save(i,:)+Myd_save(i,:);
    
%% Aerodynamics Forces and Torques


% angular velocity in the left wing frame
% px + qy + rz
Omega = 2*pi*freq*[x(i,5);x(i,6);x(i,4)];


dr = 1/LN*R;

F_tran = 0;
% F_rot = 0; In the momentum equation, F_rot is not considered.

M_tran = 0;
M_rot = 0;

for j = 1:1:LN
    r = r_hat(j)*R;
    c = c_hat(j)*c_bar;
    % Local leading edge velocity
    v = cross(Omega,[0;r;0]);
    
    v = [v(1);0;v(3)];
    
    % Local drag direction
    if norm(v) ~= 0
        dir_v = v/norm(v);                                        
    else
        dir_v = [0;0;0];
    end
    
    v_x = dot(dir_v,[1;0;0]);
    v_z = dot(dir_v,[0;0;1]);
    % Local angle of attack
    alpha = AoA(v_x, v_z);
    AoA_save(i) = alpha;
    % Local velocity direction (in the left wing frame)
    dir_drag = -dir_v;  
    % Local Lift direction (perpendicular to the flow direction)
    dir_lift = Ry( -sign(v_x)*pi/2 ) * dir_v;

    % Drag
    d_F_drag = 0.5*rho_air*(2*CD(alpha)*r_hat(j))*c*norm(v)^2*dr;
    d_F_lift = 0.5*rho_air*(2*CL(alpha)*r_hat(j))*c*norm(v)^2*dr;
    
    d_F_tran = d_F_drag*dir_drag + d_F_lift*dir_lift;

    F_tran = F_tran + d_F_tran;
    
    % translational torque
    
    dcp = 0.82/pi*abs(alpha)+0.05;         % dimensionless
    z_cop_tran = c*dcp;
    
    r_tran = [0;r;-z_cop_tran];
    
    
    d_M_tran = cross(r_tran,d_F_tran);

    M_tran = M_tran + d_M_tran;   
    
    Crot = 5;
    
    
    d_M_y_rot = -0.5*Crot*rho_air*abs(2*pi*freq*x(i,6))*2*pi*freq*x(i,6)*dr*(1/4*c^4);
    d_M_z_rot = -0.5*Crot*rho_air*abs(2*pi*freq*x(i,6))*2*pi*freq*x(i,6)*dr*(1/3*c^3);
    
    d_M_rot = [0;d_M_y_rot;d_M_z_rot];
    
    M_rot = M_rot+d_M_rot;

end


R01 = Rz(2*pi*x(i,1))*Rx(2*pi*x(i,2))*Ry(2*pi*x(i,3));
% rotational force
Crot_theo = pi*(0.75-0);


v_tip = cross(Omega,[0;R;0]);
v_tip = [v_tip(1);0;v_tip(3)];

dir_rot = [sign(x(i,6))*1;0;0];

F_rot = Crot_theo*rho_air*norm(v_tip)*abs(2*pi*freq*x(i,6))*c_bar^2*R*temp_rot*dir_rot;




% save - not sure their units
F_tran_save(i,:) =  R01*F_tran;  % in XYZ, dimensinolizee by the weight
F_rot_save(i,:) =  R01*F_rot;    % in XYZ, dimensinolizee by the weight

F_aero_save(i,:) = (F_tran + F_rot);

F_drag_save(i) = dot(F_aero_save(i,:),dir_drag);

% lift.
F_lw_save(i,:) = R01*(F_tran + F_rot);    % in XYZ
F_ltran_save(i,:) = R01*(F_tran);    % in XYZ
F_lrot_save(i,:) = R01*(F_rot);    % in XYZ

% actual lift.
F_lw(i,:) = F_lw_save(i,:);     




M_tran_save(i,:) = M_tran;
M_rot_save(i,:) = M_rot;

% dimensionless
Maero_save(i,:) = (M_tran+M_rot)/(2*pi*freq^2*Ixx);


% gravity
g = 9.81e-3;        % gravitional constant

F_g = R01\[0;0;mw*g];
r_g = [0;0.5*R;0.5*c_bar];

M_g_save(i,:) = cross(r_g,F_g)/(2*pi*freq^2*Ixx);


%% Total Torque
% only torque is in the inertial coordinate system.
M_left = tau(i,:) + Mzs_save(i,:)+Mxs_save(i,:)+Mys_save(i,:)+ Mzd_save(i,:)+Mxd_save(i,:)+Myd_save(i,:)+ Maero_save(i,:) + M_g_save(i,:);


%% Power
Omega = [x(i,5);x(i,6);x(i,4)];
% The inertial moment of the wing
Inertial_Moment = -M_left';

Pa(i) = dot(Maero_save(i,:)',Omega);
Pi(i) = dot(Inertial_Moment,Omega);
Pg(i) = dot(M_g_save(i,:)',Omega);
Ps(i) = dot(Mzs_save(i,:)'+Mxs_save(i,:)'+Mys_save(i,:)',Omega);
Pd(i) = dot(Mzd_save(i,:)'+Mxd_save(i,:)'+Myd_save(i,:)',Omega);
Pinput(i) = dot(tau(i,:)',Omega);

Psum(i) = Pa(i)+Pi(i)+Pg(i)+Ps(i)+Pd(i)+Pinput(i);


% % y
% Maero_save_0(i,:) = Ry(2*pi*x(i,3))*Maero_save(i,:)';
% % x'
% Maero_save_1(i,:) = Rx(2*pi*x(i,2))*Ry(2*pi*x(i,3))*Maero_save(i,:)';
% % z''
% Maero_save_2(i,:) = R01*Maero_save(i,:)';
% 
% ratio(i,:) = Myd(i)/Maero_save_0(i,2);

    
end




%% 

joint_parameters_plot = [kphi cphi kth cth kpsi cpsi];       
power_plot = [mean(Pa) mean(Pi) mean(Pg) mean(Ps) mean(Pd) mean(Pinput) mean(Psum(i))];
result = [mean(F_lw_save(:,3)),mean(F_lw(:,3)),mean(F_drag_save(i)),mean(F_ltran_save(:,3)),mean(F_lrot_save(:,3))];






