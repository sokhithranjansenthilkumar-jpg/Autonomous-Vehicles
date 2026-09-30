function plotComparison(results,outputDirectory)
%PLOTCOMPARISON Show paired clearance and completion time, with failure counts.
presets=unique(results.preset,'stable'); n=numel(presets);
fig=figure('Visible','off','Color','w','Position',[100 100 1200 650]);
cleanup=onCleanup(@()close(fig));
layout=tiledlayout(fig,2,n,'TileSpacing','compact');
for k=1:n
    r=results(results.preset==presets(k),:);
    for metric=1:2
        ax=nexttile(layout,k+(metric-1)*n); hold(ax,'on');
        % Export contrast must not depend on the user's MATLAB desktop theme.
        set(ax,'Color','w','XColor',[.15 .15 .15],'YColor',[.15 .15 .15], ...
            'GridColor',[.55 .55 .55]);
        if metric==1, field='minimumClearanceM'; label='Minimum conservative clearance (m)';
        else, field='timeToGoalS'; label='Completion time (s; completions only)'; end
        for seed=unique(r.seed)'
            pair=r(r.seed==seed,:); values=nan(1,2);
            for j=1:2
                modes=["baseline","risk-aware"]; entry=pair(pair.mode==modes(j),:);
                if ~isempty(entry), values(j)=entry.(field); end
            end
            plot(ax,[1 2],values,'-','Color',[.75 .78 .80]);
            scatter(ax,1,values(1),25,[.45 .49 .54],'filled');
            scatter(ax,2,values(2),25,[0 .55 .58],'filled');
        end
        if metric==1, yline(ax,0,':','contact','Color',[.65 .15 .15]); end
        xlim(ax,[.6 2.4]); xticks(ax,[1 2]); xticklabels(ax,{'Baseline','Risk-aware'});
        ylabel(ax,label,'Color',[.15 .15 .15]); grid(ax,'on');
        title(ax,sprintf('%s | n=%d; non-completions=%d',presets(k),height(r),sum(r.status~="goal_reached")), ...
            'Color',[.15 .15 .15]);
    end
end
title(layout,'Paired simulation evidence — not a road-safety validation','Color',[.15 .15 .15]);
exportgraphics(fig,fullfile(outputDirectory,'evaluation_summary.png'),'BackgroundColor','white');
clear cleanup
end
