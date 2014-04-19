% find angle of attack
function AoA = AoA(x,z)

alpha = atan2(z,x);
if x>=0 && z>=0
AoA = pi/2-alpha;
elseif x<=0 && z>=0
AoA = alpha-pi/2;
elseif x<=0 && z<=0
AoA = alpha+3*pi/2;
elseif x>=0 && z<=0
AoA = -alpha+pi/2;

end