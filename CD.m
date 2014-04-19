function CD = CD(alpha)
alpha = alpha *180/pi;       % to degrees
CD = 1.92-1.55*cos((2.04*alpha - 9.82)*pi/180);
end
