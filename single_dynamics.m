function [x_dot] = single_dynamics(t,x)

%% Global Parameters
global freq c_bar R
global Ixx Iyy Izz Iyz
global c_hat r_hat mw
global kphi cphi kth cth kpsi cpsi tauwm

global rho_air

global LN g


%% initial condition
x_dot = zeros(6,1);     % state vector

%% generalized coordinates
% stroke, z direction                  x(1)    phi
% deviation, x directoin               x(2)    th
% rotation, y direction                x(3)    psi

%% generalized speed
% stroke velocity, z direction         x(4)    r
% deviation velocity, x direction      x(5)    p
% rotation velocity, y direction       x(6)    q

%% sine,cosine function

    s2 = sin(2*pi*x(2));
    c2 = cos(2*pi*x(2));
    s3 = sin(2*pi*x(3));
    c3 = cos(2*pi*x(3));
    % Constraint Equation
    % wing angular velocity
    W10 = [c2*c3 s3 0;
          -c2*s3 c3 0;
           s2 0 1];
    temp = W10\[x(4);x(5);x(6)];

    x_dot(1) = temp(1);
    x_dot(2) = temp(2);
    x_dot(3) = temp(3);
    



%% wing moment of inertia matrix
I_wing = [  1   0      0;
            0   Iyy/Ixx    Iyz/Ixx;
            0   Iyz/Ixx    Izz/Ixx];


%% Hinge Torques
% Active hinge torques, along z direction
% input signal
tau = tauwm * sin(2*pi*t);
Minput = transpose(Ry(2*pi*x(3)))*transpose(Rx(2*pi*x(2)))*[0;0;tau];

% stroke
Mzs = -kphi*x(1);
Mzd = -cphi*x_dot(1);
Mzspring = transpose(Ry(2*pi*x(3)))*[0;0;Mzs];
Mzdamping = transpose(Ry(2*pi*x(3)))*[0;0;Mzd];

% Passive hinge torques, along x direction
% deviation
Mxs = -kth*x(2);
Mxd = -cth*x_dot(2);
Mxspring = transpose(Ry(2*pi*x(3)))*[Mxs;0;0];
Mxdamping = transpose(Ry(2*pi*x(3)))*[Mxd;0;0];

% Passive hinge torques, along y direction
% rotation
Mys = -kpsi*x(3);
Myd = -cpsi*x_dot(3);
Myspring = [0;Mys;0];
Mydamping = [0;Myd;0];


Mspring = Mzspring+Mxspring+Myspring;
Mdamping = Mzdamping+Mxdamping+Mydamping;

%% Aerodynamics Forces and Torques
% angular velocity in the left wing frame
% px + qy + rz
Omega = 2*pi*freq*[x(5);x(6);x(4)];


dr = 1/LN*R;

M_tran = 0;
M_rot = 0;

for i = 1:1:LN
    r = r_hat(i)*R;
    c = c_hat(i)*c_bar;
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
    % Local velocity direction (in the left wing frame)
    dir_drag = -dir_v;  
    % Local Lift direction (perpendicular to the flow direction)
    dir_lift = Ry( -sign(v_x)*pi/2 ) * dir_v;

    % Drag
    d_F_drag = 0.5*rho_air*(2*CD(alpha)*r_hat(i))*c*norm(v)^2*dr;
    d_F_lift = 0.5*rho_air*(2*CL(alpha)*r_hat(i))*c*norm(v)^2*dr;
    
    d_F_tran = d_F_drag*dir_drag + d_F_lift*dir_lift;

    % translational torque
    
    dcp = 0.82/pi*abs(alpha)+0.05;         % dimensionless
    z_cop_tran = c*dcp;
    
    r_tran = [0;r;-z_cop_tran];
    
    
    d_M_tran = cross(r_tran,d_F_tran);

    M_tran = M_tran + d_M_tran;   
    
    Crot = 5;
    %dir_rot = [1;0;0];
    %d_F_rot = 0.5*Crot*rho_air*abs(x(6))*x(6)*dr*(1/3*(c_hat(i)*c_bar)^3)*dir_rot;
    
    d_M_y_rot = -0.5*Crot*rho_air*abs(2*pi*freq*x(6))*2*pi*freq*x(6)*dr*(1/4*c^4);
    d_M_z_rot = -0.5*Crot*rho_air*abs(2*pi*freq*x(6))*2*pi*freq*x(6)*dr*(1/3*c^3);
    
    d_M_rot = [0;d_M_y_rot;d_M_z_rot];
    
    M_rot = M_rot+d_M_rot;

end




Maero = M_tran+M_rot;

R01 = Rz(2*pi*x(1))*Rx(2*pi*x(2))*Ry(2*pi*x(3));
g = 9.81e-3;        % gravitional constant
 
F_g = R01\[0;0;mw*g];
r_g = [0;0.5*R;0.5*c_bar];
 
M_g = cross(r_g,F_g);





%% Total Torque
% only torque is in the inertial coordinate system.
% all of these are dimensionless numbers.
M_left = Minput + Mspring + Mdamping + Maero/(2*pi*freq^2*Ixx) +M_g/(2*pi*freq^2*Ixx);

%% Setup ODEs
% in left wing coordinate
Omega = [x(5);x(6);x(4)];
ODE = I_wing\(M_left - 2*pi*cross(Omega,I_wing*Omega));
x_dot(5) = ODE(1);
x_dot(6) = ODE(2);
x_dot(4) = ODE(3);

%% plot instanteous aerodynamic force
% figure(1)
% subplot(3,1,1)
% hold on
% grid on
% plot(t,x(1)*180/pi,'b')
% ylabel('stroke angle')
% subplot(3,1,2)
% hold on
% grid on
% plot(t,x(2)*180/pi,'r')
% ylabel('deviation angle')
% subplot(3,1,3)
% hold on
% grid on
% plot(t,x(3)*180/pi,'g')
% ylabel('rotation angle')
%


end















