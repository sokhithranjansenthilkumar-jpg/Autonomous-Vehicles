function overlap = circleOverlap(centreA, radiusA, centreB, radiusB)
%CIRCLEOVERLAP True when two closed circular footprints intersect.
delta = centreA - centreB;
overlap = sum(delta.^2, 2) <= (radiusA + radiusB).^2;
end
