function label = ttcLabel(ttc,horizon)
%TTCLABEL Never confuse missing data with no predicted collision.
if isnan(ttc)
    label="unavailable";
elseif isinf(ttc)
    label=sprintf('none within %.1f s',horizon);
else
    label=sprintf('%.2f s (%.1f s horizon)',ttc,horizon);
end
end
