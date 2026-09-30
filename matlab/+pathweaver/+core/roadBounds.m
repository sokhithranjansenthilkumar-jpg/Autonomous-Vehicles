function [leftY, rightY] = roadBounds(xWorldM, sc)
%ROADBOUNDS Slightly irregular, unmarked drivable road boundaries.
centreY = 0.18*sin(xWorldM/15) + 0.08*sin(xWorldM/5.5);
halfWidth = sc.nominalWidthM/2 + 0.18*sin(xWorldM/21 + 0.6);
leftY = centreY + halfWidth;
rightY = centreY - halfWidth;
end
