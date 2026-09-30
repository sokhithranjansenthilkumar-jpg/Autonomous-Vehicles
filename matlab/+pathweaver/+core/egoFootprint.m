function [centres, radius] = egoFootprint(positions, headings, cfg)
%EGOFOOTPRINT Three covering discs about the virtual bicycle reference point.
% Each disc covers one third of the declared L-by-W rectangular body.
offsets = [-1 0 1]*cfg.ego.lengthM/3;
radius = hypot(cfg.ego.lengthM/6, cfg.ego.widthM/2);
centres = zeros(size(positions,1),2,3);
for k=1:3
    centres(:,:,k) = positions + offsets(k)*[cos(headings(:)) sin(headings(:))];
end
end
