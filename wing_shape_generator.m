%% wing shape is generated based on BETA distribution from Ellington
% input R, AR, r1_S
% dimensionless radius of wing moments of inertia
r2 = 0.929*r1^(0.732);

% mean wing chord
c_bar = (2*R)/AR; 


% BETA distribution parameters
p = r1 *(r1*(1-r1)/(r2^2-r1^2)-1);
q = (1-r1) *(r1*(1-r1)/(r2^2-r1^2)-1);

% number of point along wing leading edge axis
N = 1000;
dx = 1/N;

% beta-function parameters
BETA = 0;
for i = 1 :N
    x = i/N;
    BETA = BETA + x^(p-1)*(1-x)^(q-1)*dx;
end

% dimensionless chord length calculated from beta distribution
for i = 1 :N
    x = i/N;
    c(i) = x^(p-1)*(1-x)^(q-1)/BETA;
end

figure(1)
hold on
r = 1/N:dx:1;
plot(-r*R,-c*c_bar,'k','linewidth',3)
plot([0 -R-10],[0,0],'k','linewidth',3)
axis equal



















