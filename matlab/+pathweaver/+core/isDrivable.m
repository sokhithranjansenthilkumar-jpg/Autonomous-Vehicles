function inside = isDrivable(positionWorldM, marginM, sc)
%ISDRIVABLE Test points against explicit road free-space boundaries.
x = positionWorldM(:,1);
y = positionWorldM(:,2);
[leftY, rightY] = pathweaver.core.roadBounds(x, sc);
inside = x >= 0 & x <= sc.roadLengthM & ...
    y <= leftY - marginM & y >= rightY + marginM;
end
