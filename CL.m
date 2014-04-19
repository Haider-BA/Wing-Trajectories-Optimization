function CL = CL(alpha)
alpha = alpha *180/pi;       % to degrees
CL = 0.225 + 1.58*sin((2.13*alpha - 7.2)*pi/180);
end