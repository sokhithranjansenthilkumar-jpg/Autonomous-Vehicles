function angle = wrapAngle(angle)
%WRAPANGLE Wrap radians to [-pi, pi] without toolbox dependencies.
angle = mod(angle + pi, 2*pi) - pi;
end
