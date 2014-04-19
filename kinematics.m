clear all;
close all;

n = 2000;

x = zeros(n,6);

dt = 2*pi/n;
t = 0:dt:2*pi;

wingbeats = 1;



change = pi/180;
th_0_val = 0;
phi_a0 = 90*change;
phi_b0 = 45*change;
th_0 = 0*change;
th_a0 = 40*change;
%th_b0 = (phi_b0 + th_0_val)*change;
psi_a0 = 50*change;
psi_b0 = phi_b0 - 90*change;

for j = 1:1:1
    

    th_b0 = phi_b0 + th_0_val;


    for i = 1:1:n
        x(i,1) = phi_a0*cos(t(i) + phi_b0);
        x(i,2) = th_0 + th_a0*sin(t(i) + th_b0);
        x(i,3) = psi_a0*cos(t(i) + psi_b0);
    end

    animation_wing_trajectory( x, n , 50, 0.2353, j);
    
    th_0_val = th_0_val + 360/36*change;

end






